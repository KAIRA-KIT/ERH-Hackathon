#!/usr/bin/env python3
"""Regenerate config/trunk_geometry.yaml from the STL files (run manually).

Per element it measures the tendon hole radius, the axial extent of the holes
and (where a clean sphere fit exists) the distance from socket centre to ball
centre. Links whose ball/socket fit is poor use a linear extrapolation of the
pitch. All lengths are in mm, in the mesh frame.
"""
import struct
import sys
from pathlib import Path

import numpy as np
import yaml

PKG = Path(__file__).resolve().parent.parent
NAMES = [f'Link{i}' for i in range(1, 10)] + ['EEF']
HOLE_ANGLES = [90, 30, -30, -90, -150, 150]  # hole 1..6, clockwise seen from the tip


def load_stl(path):
    d = path.read_bytes()
    n = struct.unpack('<I', d[80:84])[0]
    dt = np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')])
    return np.frombuffer(d, dtype=dt, count=n, offset=84)['v'].reshape(-1, 3)


def fit_sphere(p):
    A = np.c_[2 * p, np.ones(len(p))]
    x, *_ = np.linalg.lstsq(A, (p ** 2).sum(1), rcond=None)
    c = x[:3]
    r = np.sqrt(x[3] + c @ c)
    return c, r, np.abs(np.linalg.norm(p - c, axis=1) - r).max()


def hole_info(p):
    """Radius (mm) and z-extent of the six 1 mm holes."""
    xy = p[:, :2]
    radii = np.hypot(*xy.T)
    # centre of the 0.5 mm radius hole walls: vertices lying on small circles
    best = None
    for r in np.arange(5.0, 25.0, 0.1):
        c = np.array([0.0, r])
        m = np.hypot(*(xy - c).T) < 0.6
        if m.sum() > 20 and (best is None or m.sum() > best[1]):
            best = (r, m.sum())
    r_hole = round(best[0] * 2) / 2
    zs = []
    for a in HOLE_ANGLES:
        c = r_hole * np.array([np.cos(np.radians(a)), np.sin(np.radians(a))])
        m = np.hypot(*(xy - c).T) < 0.8
        zs.append((p[m, 2].min(), p[m, 2].max()))
    del radii
    return r_hole, float(np.median([z[0] for z in zs])), float(np.median([z[1] for z in zs]))


def main(mesh_dir=PKG / 'meshes', out=PKG / 'config' / 'trunk_geometry.yaml'):
    elements = []
    for name in NAMES:
        p = np.unique(load_stl(Path(mesh_dir) / f'{name}.stl').round(4), axis=0)
        zmin, zmax = p[:, 2].min(), p[:, 2].max()
        rxy = np.hypot(p[:, 0], p[:, 1])
        r_hole, h_lo, h_hi = hole_info(p)
        ball = socket = None
        top = (p[:, 2] > zmax - 6) & (rxy < 6)
        bot = (p[:, 2] < zmin + 4) & (rxy < 6)
        if top.sum() > 20:
            c, r, err = fit_sphere(p[top])
            if err < 0.8 and r > 3:
                ball = c[2]
        if bot.sum() > 20:
            c, r, err = fit_sphere(p[bot])
            if err < 0.2:
                socket = c[2]
        elements.append(dict(name=name, mesh=f'{name}.stl', r_hole=float(r_hole),
                             hole_lo=h_lo, hole_hi=h_hi, ball=ball, socket=socket))
    # pitch = socket centre to ball centre; extrapolate where the fit was poor
    good = [(i, e['ball'] - e['socket']) for i, e in enumerate(elements[:9])
            if e['ball'] is not None and e['socket'] is not None]
    slope, icpt = np.polyfit([i for i, _ in good], [v for _, v in good], 1)
    for i, e in enumerate(elements[:9]):
        if e['socket'] is None:
            e['socket'] = (e['ball'] if e['ball'] is not None else 0.0) - (slope * i + icpt)
        if e['ball'] is None:
            e['ball'] = e['socket'] + slope * i + icpt
    elements[9]['socket'] = elements[9]['socket'] if elements[9]['socket'] is not None \
        else elements[9]['hole_lo'] - (elements[8]['hole_lo'] - elements[8]['socket'])
    out_elems = []
    for i, e in enumerate(elements):
        origin = e['socket']  # element frame origin = its socket centre
        out_elems.append(dict(
            name=e['name'], mesh=e['mesh'], r_hole=float(round(e['r_hole'], 2)),
            hole_z_in=float(round(e['hole_lo'] - origin, 2)),
            hole_z_out=float(round(e['hole_hi'] - origin, 2)),
            pitch=float(round(e['ball'] - e['socket'], 2)) if i < 9 else 0.0,
            mesh_origin_z=float(round(origin, 2))))
    geo = dict(
        spool_radius_mm=6.0,
        hole_angles_deg=HOLE_ANGLES,
        tendon_anchor_element=[10, 5, 10, 5, 10, 5],  # tendon i -> servo i
        segment_split_element=5,
        base=dict(r_hole=out_elems[0]['r_hole'], hole_z_out=out_elems[0]['hole_z_in']),
        elements=out_elems)
    Path(out).write_text(yaml.safe_dump(geo, sort_keys=False))
    print(f'wrote {out}')


if __name__ == '__main__':
    main(*sys.argv[1:])
