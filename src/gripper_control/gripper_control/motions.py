"""Demo poses and small motion helpers. Angles in radians, q = (theta_p, phi_p, theta_d, phi_d)."""
import numpy as np

D = np.radians


def _static(tp, pp, td, pd):
    q = np.array([D(tp), D(pp), D(td), D(pd)])
    return lambda t: q


def _lasso(t, period=8.0):
    phi = 2 * np.pi * t / period
    return np.array([D(30), phi, D(60), phi])


def _look_around(t, period=8.0):
    return np.array([D(45), 2 * np.pi * t / period, 0.0, 0.0])


# name -> (duration in s, q(t)); the rate limiter in the node handles the travel
DEMOS = {
    'straight': (4.0, _static(0, 0, 0, 0)),
    'c_curve': (6.0, _static(60, 0, 60, 0)),
    's_curve': (6.0, _static(60, 0, 90, 180)),
    'hook': (6.0, _static(0, 0, 120, 0)),
    'curl': (6.0, _static(100, 0, 125, 0)),
    'lasso': (10.0, _lasso),
    'look_around': (10.0, _look_around),
}


def clamp_q(q, max_segment_bend):
    q = np.array(q, float)
    q[0] = np.clip(q[0], 0.0, max_segment_bend)
    q[2] = np.clip(q[2], 0.0, max_segment_bend)
    return q


def limit_step(u, u_des, max_step):
    """Move bend vectors u toward u_des by at most max_step rad per segment."""
    u = np.array(u, float)
    for k in (0, 2):
        d = u_des[k:k + 2] - u[k:k + 2]
        n = np.linalg.norm(d)
        u[k:k + 2] += d if n <= max_step else d * (max_step / n)
    return u
