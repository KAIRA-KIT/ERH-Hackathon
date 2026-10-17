"""Piecewise-constant-curvature kinematics of the two-segment tendon trunk.

q = (theta_p, phi_p, theta_d, phi_d) in radians: total bend and bend direction
(azimuth in the base frame) of the proximal and distal segment. Each segment's
bend is shared equally by its ball joints. Tendon lengths come from routing
each line through its holes, so taper, joint pitch and stage coupling are
handled by the geometry. Lengths are in mm.
"""
import numpy as np


def q_to_u(q):
    """Smooth 4-vector (bend vectors) for a q; avoids the phi singularity at 0."""
    return np.array([q[0] * np.cos(q[1]), q[0] * np.sin(q[1]),
                     q[2] * np.cos(q[3]), q[2] * np.sin(q[3])])


def u_to_q(u):
    return np.array([np.hypot(u[0], u[1]), np.arctan2(u[1], u[0]),
                     np.hypot(u[2], u[3]), np.arctan2(u[3], u[2])])


def bend_rotation(phi, angle):
    """Rotate by `angle` about (-sin phi, cos phi, 0): tilts +z toward azimuth phi."""
    a = np.array([-np.sin(phi), np.cos(phi), 0.0])
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)


def joint_rotations(q, geo):
    """Rotation of each joint 1..num_joints (list index 0 = joint 1)."""
    n_prox = geo.split
    n_dist = geo.num_joints - geo.split
    rots = []
    for j in range(1, geo.num_joints + 1):
        if geo.segment_of_joint(j) == 0:
            rots.append(bend_rotation(q[1], q[0] / n_prox))
        else:
            rots.append(bend_rotation(q[3], q[2] / n_dist))
    return rots


def element_poses(q, geo):
    """4x4 pose of every element frame (origin = socket centre), base first."""
    poses = [np.eye(4)]
    for j, R in enumerate(joint_rotations(q, geo), start=1):
        T = poses[-1].copy()
        T[:3, 3] += T[:3, :3] @ np.array([0.0, 0.0, geo.elements[j - 1]['pitch']])
        T[:3, :3] = T[:3, :3] @ R
        poses.append(T)
    return poses


def tendon_lengths(q, geo):
    """Free length of each tendon, base to its anchor element (mm)."""
    poses = element_poses(q, geo)
    lengths = np.zeros(geo.num_tendons)
    for i in range(geo.num_tendons):
        for j in range(1, geo.anchors[i] + 1):
            a = poses[j - 1] @ np.append(geo.hole_point(j - 1, i, True), 1.0)
            b = poses[j] @ np.append(geo.hole_point(j, i, False), 1.0)
            lengths[i] += np.linalg.norm(b - a)
    return lengths


def servo_angles(q, geo, signs=None):
    """Servo angles (deg, positive = pull line in) that realise q from straight."""
    signs = np.ones(geo.num_tendons) if signs is None else np.asarray(signs, float)
    dl = tendon_lengths(np.zeros(4), geo) - tendon_lengths(q, geo)
    return signs * np.degrees(dl / geo.spool_radius)


def estimate_q(angles, geo, signs=None, u0=None, iterations=30):
    """Least-squares q (rad) from servo angles (deg); damped Gauss-Newton on bend vectors."""
    signs = np.ones(geo.num_tendons) if signs is None else np.asarray(signs, float)
    angles = np.asarray(angles, float)
    ok = np.isfinite(angles)
    u = np.zeros(4) if u0 is None else np.array(u0, float)

    def residual(u_):
        return (servo_angles(u_to_q(u_), geo, signs) - angles)[ok]

    lam = 1e-3
    r = residual(u)
    for _ in range(iterations):
        J = np.empty((ok.sum(), 4))
        for k in range(4):
            du = np.zeros(4)
            du[k] = 1e-5
            J[:, k] = (residual(u + du) - r) / 1e-5
        step = np.linalg.solve(J.T @ J + lam * np.eye(4), -J.T @ r)
        r_new = residual(u + step)
        if r_new @ r_new < r @ r:
            u, r, lam = u + step, r_new, lam * 0.3
            if np.linalg.norm(step) < 1e-7:
                break
        else:
            lam *= 10
    return u_to_q(u), u
