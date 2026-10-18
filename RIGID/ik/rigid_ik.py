"""
RIGID inverse kinematics for the Raspberry Pi (same model as the simulator).

Base frame: origin = centre of the socket on the lid (first ball joint),
            z = along the straight arm, pointing AWAY from the base (down, when hanging),
            x/y = horizontal; tendon k's hole sits at 43.6 + 60*k deg from +x.
Units: mm and degrees at the interface.

    from rigid_ik import solve, send
    shape, tip, err = solve(target_xyz=(90, 40, 235))   # mm in the base frame
    send(ser, shape)                                    # ser = serial.Serial('/dev/ttyUSB0', 115200)

Needs: numpy (pip install numpy pyserial)
"""
import math
import numpy as np

LAYOUT_TWO = True     # True: tendons 0,2,4 end at Link4 (mid); 1,3,5 at the tip
SPLIT = 5             # last joint of section 1 (bodies 0..5 = Link-1..Link4)

# per body: joint-to-joint length d (mm), max tilt per joint (deg)  — from the CAD
D = [29.26, 28.168, 27.077, 25.985, 24.893, 23.801, 22.709, 21.617, 21.493, 19.389, 18.038, 15.458]
TILT = [25.0, 26.5, 27.5, 28.0, 28.5, 29.5, 30.5, 31.5, 33.0, 35.5, 36.5, 36.0]
NJ = len(D)


def sec_of(j):
    return 0 if (not LAYOUT_TWO or j <= SPLIT) else 1


def sec_joints(s):
    return [j for j in range(NJ) if sec_of(j) == s]


N_SEC = 2 if LAYOUT_TWO else 1
B_MAX = [0.95 * len(sec_joints(s)) * min(TILT[j] for j in sec_joints(s)) * math.pi / 180 for s in range(N_SEC)]


def rotvec(w):
    th = np.linalg.norm(w)
    if th < 1e-12:
        return np.eye(3)
    k = w / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K


def tip_of(q):
    """q = [bx0, by0, bx1, by1] (rad, total bend vector per section) -> tip position (mm)."""
    R, p = np.eye(3), np.zeros(3)
    n = [len(sec_joints(s)) for s in range(N_SEC)]
    for j in range(NJ):
        s = sec_of(j)
        bx, by = q[2 * s] / n[s], q[2 * s + 1] / n[s]
        R = R @ rotvec(np.array([-by, bx, 0.0]))
        p = p + R @ np.array([0, 0, D[j]])
    return p


def clamp(q):
    q = np.array(q, float)
    for s in range(N_SEC):
        b = math.hypot(q[2 * s], q[2 * s + 1])
        if b > B_MAX[s]:
            q[2 * s:2 * s + 2] *= B_MAX[s] / b
    q[2 * N_SEC:] = 0
    return q


def ik(target, q0=None, iters=40, lam2=30.0):
    """Damped least squares, same as the simulator."""
    q = clamp(q0 if q0 is not None else [0.01, 0.01, 0, 0])
    target = np.asarray(target, float)
    nq = 2 * N_SEC
    for _ in range(iters):
        tip = tip_of(q)
        e = target - tip
        if np.linalg.norm(e) < 0.05:
            break
        J = np.zeros((3, nq))
        for i in range(nq):
            dq = q.copy(); dq[i] += 1e-4
            J[:, i] = (tip_of(dq) - tip) / 1e-4
        q[:nq] += J.T @ np.linalg.solve(J @ J.T + lam2 * np.eye(3), e)
        q = clamp(q)
    return q


def q_to_shape(q):
    """-> (b1, phi1, b2, phi2) in degrees, the format the ESP32 'S' command expects."""
    out = []
    for s in range(2):
        bx, by = q[2 * s], q[2 * s + 1]
        out += [math.degrees(math.hypot(bx, by)), math.degrees(math.atan2(by, bx)) % 360]
    return out


_last_q = None


def solve(target_xyz):
    """target (mm, base frame) -> (shape degrees, reached tip, error mm). Warm-starts from the last call."""
    global _last_q
    q = ik(target_xyz, _last_q)
    _last_q = q
    tip = tip_of(q)
    return q_to_shape(q), tip, float(np.linalg.norm(tip - np.asarray(target_xyz)))


def send(ser, shape):
    b1, p1, b2, p2 = shape
    ser.write(f"S {b1:.1f} {p1:.1f} {b2:.1f} {p2:.1f}\n".encode())


if __name__ == "__main__":
    for t in [(90, 40, 235), (0, 0, 277.9), (120, 0, 200), (-60, 80, 240)]:
        shape, tip, err = solve(t)
        print(f"target {t} -> S {shape[0]:.1f} {shape[1]:.1f} {shape[2]:.1f} {shape[3]:.1f}   tip {np.round(tip, 1)}  err {err:.2f} mm")
