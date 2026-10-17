"""URDF generation for RViz. Each ball joint is three revolute joints (x, y, z)."""
import numpy as np

MM = 0.001


def joint_names(geo):
    return [f'j{j}_{axis}' for j in range(1, geo.num_joints + 1) for axis in 'xyz']


def euler_xyz(R):
    """Angles (a, b, c) with R = Rx(a) Ry(b) Rz(c)."""
    b = np.arcsin(np.clip(R[0, 2], -1.0, 1.0))
    return np.arctan2(-R[1, 2], R[2, 2]), b, np.arctan2(-R[0, 1], R[0, 0])


def generate(geo, base_height=0.4, mesh_uri='package://gripper_control/meshes'):
    out = ['<?xml version="1.0"?>', '<robot name="trunk">',
           '<material name="grey"><color rgba="0.75 0.75 0.78 1"/></material>',
           '<link name="world"/>']
    # the trunk hangs from the table: base frame +z (toward the tip) points down
    out.append('<link name="elem_0"><visual><geometry>'
               '<cylinder radius="0.03" length="0.01"/></geometry>'
               '<material name="grey"/></visual></link>')
    out.append(f'<joint name="hang" type="fixed"><parent link="world"/><child link="elem_0"/>'
               f'<origin xyz="0 0 {base_height}" rpy="{np.pi} 0 0"/></joint>')
    for j in range(1, geo.num_joints + 1):
        parent = f'elem_{j - 1}'
        pitch = geo.elements[j - 1]['pitch'] * MM
        chain = [parent, f'j{j}_a', f'j{j}_b', f'elem_{j}']
        for k, axis in enumerate('xyz'):
            origin = f'<origin xyz="0 0 {pitch}"/>' if k == 0 else ''
            out.append(
                f'<joint name="j{j}_{axis}" type="revolute"><parent link="{chain[k]}"/>'
                f'<child link="{chain[k + 1]}"/>{origin}<axis xyz="{" ".join("1" if c == axis else "0" for c in "xyz")}"/>'
                f'<limit lower="-1.6" upper="1.6" effort="1" velocity="1"/></joint>')
        e = geo.elements[j]
        out.append(f'<link name="j{j}_a"/><link name="j{j}_b"/>')
        out.append(
            f'<link name="elem_{j}"><visual>'
            f'<origin xyz="0 0 {-e["mesh_origin_z"] * MM}"/>'
            f'<geometry><mesh filename="{mesh_uri}/{e["mesh"]}" scale="{MM} {MM} {MM}"/></geometry>'
            f'<material name="grey"/></visual></link>')
    out.append('</robot>')
    return '\n'.join(out)
