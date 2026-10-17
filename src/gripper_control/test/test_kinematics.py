import numpy as np

from gripper_control import kinematics as kin
from gripper_control.geometry import Geometry

geo = Geometry()
D = np.radians


def test_straight_gives_zero_servo_angles():
    assert np.allclose(kin.servo_angles(np.zeros(4), geo), 0.0)


def test_bending_toward_hole_shortens_it_and_lengthens_opposite():
    L0 = kin.tendon_lengths(np.zeros(4), geo)
    for hole in range(6):
        phi = geo.hole_angles[hole]
        L = kin.tendon_lengths(np.array([D(40), phi, 0.0, 0.0]), geo)
        opposite = (hole + 3) % 6
        assert L[hole] < L0[hole]
        assert L[opposite] > L0[opposite]


def test_distal_bend_leaves_stage_two_unchanged():
    L0 = kin.tendon_lengths(np.zeros(4), geo)
    L = kin.tendon_lengths(np.array([0.0, 0.0, D(60), D(30)]), geo)
    assert np.allclose(L[[1, 3, 5]], L0[[1, 3, 5]])
    assert not np.allclose(L[[0, 2, 4]], L0[[0, 2, 4]])


def test_servo_angles_are_pull_in_for_bend_toward_hole():
    a = kin.servo_angles(np.array([D(30), geo.hole_angles[0], 0.0, 0.0]), geo)
    assert a[0] > 0 > a[3]


def test_estimate_round_trip():
    q = np.array([D(50), D(70), D(80), D(-120)])
    a = kin.servo_angles(q, geo)
    q_est, _ = kin.estimate_q(a, geo)
    assert np.allclose(kin.servo_angles(q_est, geo), a, atol=0.05)
    assert np.allclose(q_est, q, atol=D(0.5))


def test_estimate_ignores_missing_servo():
    q = np.array([D(30), D(10), D(60), D(40)])
    a = kin.servo_angles(q, geo)
    a[2] = np.nan
    q_est, _ = kin.estimate_q(a, geo)
    assert np.allclose(q_est, q, atol=D(1.0))
