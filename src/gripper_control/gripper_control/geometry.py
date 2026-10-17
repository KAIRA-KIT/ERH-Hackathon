"""Trunk geometry loaded from config/trunk_geometry.yaml (lengths in mm)."""
from pathlib import Path

import numpy as np
import yaml


def default_path():
    try:
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory('gripper_control')) / 'config' / 'trunk_geometry.yaml'
        if path.exists():
            return path
    except Exception:
        pass
    return Path(__file__).resolve().parent.parent / 'config' / 'trunk_geometry.yaml'


class Geometry:
    def __init__(self, path=None):
        with open(path or default_path()) as f:
            data = yaml.safe_load(f)
        self.spool_radius = float(data['spool_radius_mm'])
        self.hole_angles = np.radians(data['hole_angles_deg'])
        self.anchors = list(data['tendon_anchor_element'])
        self.split = int(data['segment_split_element'])
        # element 0 is the fixed base, 1..N the links, N the end effector
        base = data['base']
        self.elements = [dict(name='base', mesh=None, r_hole=base['r_hole'],
                              hole_z_in=0.0, hole_z_out=base['hole_z_out'],
                              pitch=0.0, mesh_origin_z=0.0)] + data['elements']
        self.num_joints = len(self.elements) - 1
        self.num_tendons = len(self.hole_angles)

    def hole_point(self, element, tendon, exit_side):
        """Tendon hole point in the element frame (origin = socket centre)."""
        e = self.elements[element]
        a = self.hole_angles[tendon]
        z = e['hole_z_out'] if exit_side else e['hole_z_in']
        return np.array([e['r_hole'] * np.cos(a), e['r_hole'] * np.sin(a), z])

    def segment_of_joint(self, joint):
        """0 = proximal, 1 = distal (joints are numbered 1..num_joints)."""
        return 0 if joint <= self.split else 1
