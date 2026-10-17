import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, Float64, Float64MultiArray, String

from gripper_control import kinematics as kin
from gripper_control import motions, urdf
from gripper_control.geometry import Geometry


class GripperControlNode(Node):
    def __init__(self):
        super().__init__('gripper_control')
        self.declare_parameter('driver_ns', '/trunk_control')
        self.declare_parameter('dry_run', False)
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('max_bend_rate', 30.0)       # deg/s per segment
        self.declare_parameter('max_joint_bend', 25.0)      # deg per ball joint
        self.declare_parameter('contact_load_pct', 15.0)    # load rise that counts as contact
        self.declare_parameter('squeeze_deg', 30.0)         # extra pull on the inside lines when holding
        self.declare_parameter('servo_signs', [1.0] * 6)

        self.geo = Geometry()
        ns = self.get_parameter('driver_ns').value.rstrip('/')
        self.dry_run = self.get_parameter('dry_run').value
        self.dt = 1.0 / self.get_parameter('rate_hz').value
        self.rate = np.radians(self.get_parameter('max_bend_rate').value)
        self.max_segment_bend = np.radians(self.get_parameter('max_joint_bend').value) * min(
            self.geo.split, self.geo.num_joints - self.geo.split)
        self.contact_load = self.get_parameter('contact_load_pct').value
        self.squeeze = self.get_parameter('squeeze_deg').value
        self.signs = np.array(self.get_parameter('servo_signs').value, float)
        self.joint_names = urdf.joint_names(self.geo)

        self.state = {'position': None, 'load': None}
        for name in self.state:
            self.create_subscription(
                Float64MultiArray, f'{ns}/{name}',
                lambda msg, name=name: self.state.__setitem__(name, np.array(msg.data)), 10)

        self.command_pub = self.create_publisher(Float64MultiArray, f'{ns}/command', 10)
        self.zero_pub = self.create_publisher(Empty, f'{ns}/zero', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.state_pub = self.create_publisher(String, '~/state', 10)
        self.q_est_pub = self.create_publisher(Float64MultiArray, '~/q_est', 10)

        self.create_subscription(Float64MultiArray, '~/pose', self.on_pose, 10)
        self.create_subscription(Float64, '~/grasp', self.on_grasp, 10)
        self.create_subscription(Empty, '~/release', lambda _: self.go_pose(np.zeros(4)), 10)
        self.create_subscription(Empty, '~/stop', self.on_stop, 10)
        self.create_subscription(String, '~/demo', self.on_demo, 10)
        self.create_subscription(Empty, '~/zero', self.on_zero, 10)

        self.u = np.zeros(4)            # current bend vectors (rad), what is commanded
        self.u_des = np.zeros(4)
        self.u_est = np.zeros(4)
        self.active = False             # only stream commands once something was requested
        self.mode = 'idle'
        self.phase = ''
        self.t = 0.0                    # time in the current mode
        self.demo_queue = []
        self.squeeze_vec = np.zeros(6)
        self.grasp = {}
        self.create_timer(self.dt, self.step)

    # ---- requests -------------------------------------------------------
    def set_mode(self, mode, phase=''):
        self.mode, self.phase, self.t = mode, phase, 0.0
        self.squeeze_vec = np.zeros(6)
        self.active = True

    def go_pose(self, q):
        self.set_mode('pose')
        self.u_des = kin.q_to_u(motions.clamp_q(q, self.max_segment_bend))

    def on_pose(self, msg):
        if len(msg.data) != 4:
            self.get_logger().warn('pose needs [theta_p, phi_p, theta_d, phi_d] in degrees')
            return
        self.go_pose(np.radians(msg.data))

    def on_stop(self, _):
        self.u_des = self.u.copy()
        self.set_mode('idle')

    def on_zero(self, _):
        self.zero_pub.publish(Empty())
        self.u = np.zeros(4)
        self.u_des = np.zeros(4)
        self.u_est = np.zeros(4)
        self.active = False
        self.mode = 'idle'
        self.get_logger().info('Zeroed: current pose is now the straight reference')

    def on_demo(self, msg):
        name = msg.data.strip()
        if name == 'all':
            self.demo_queue = list(motions.DEMOS)
        elif name in motions.DEMOS:
            self.demo_queue = [name]
        else:
            self.get_logger().warn(f'Unknown demo {name!r}; choose from {list(motions.DEMOS)} or all')
            return
        self.set_mode('demo', self.demo_queue.pop(0))

    def on_grasp(self, msg):
        phi = np.radians(msg.data)
        load = self.state['load']
        baseline = np.abs(load) if load is not None else None
        if baseline is None and not self.dry_run:
            self.get_logger().warn('No load feedback yet; grasp will not detect contact')
        self.grasp = dict(phi=phi, baseline=baseline, hits=0)
        self.u_des = self.u.copy()
        self.set_mode('grasp', 'curl_distal')

    # ---- control loop ---------------------------------------------------
    def inside(self, stage, phi):
        """Tendon indices of a stage (0 = stage 1, 1 = stage 2) on the inside of a bend toward phi."""
        return [i for i in range(stage, 6, 2) if np.cos(self.geo.hole_angles[i] - phi) > 0.5]

    def contact(self, stage):
        load, base = self.state['load'], self.grasp['baseline']
        if load is None or base is None or len(load) != 6:
            return False
        idx = self.inside(stage, self.grasp['phi'])
        rise = np.nanmax(np.abs(load[idx]) - base[idx]) if idx else 0.0
        self.grasp['hits'] = self.grasp['hits'] + 1 if rise > self.contact_load else 0
        return self.grasp['hits'] >= 3

    def update_target(self):
        if self.mode == 'demo':
            duration, fn = motions.DEMOS[self.phase]
            self.u_des = kin.q_to_u(motions.clamp_q(fn(min(self.t, duration)), self.max_segment_bend))
            if self.t >= duration:
                if self.demo_queue:
                    self.set_mode('demo', self.demo_queue.pop(0))
                else:
                    self.mode, self.phase = 'pose', ''
        elif self.mode == 'grasp':
            self.update_grasp()

    def update_grasp(self):
        phi, step = self.grasp['phi'], self.rate * self.dt
        if self.phase == 'curl_distal':
            theta = np.hypot(*self.u[2:])
            if self.contact(0) or theta >= self.max_segment_bend - 1e-3:
                self.u_des[2:] = self.u[2:]
                self.grasp['hits'] = 0
                self.phase = 'curl_proximal'
            else:
                self.u_des[2:] = min(theta + 2 * step, self.max_segment_bend) * np.array([np.cos(phi), np.sin(phi)])
        elif self.phase == 'curl_proximal':
            theta = np.hypot(*self.u[:2])
            in_contact = self.contact(1)
            if in_contact or theta >= self.max_segment_bend - 1e-3:
                if not in_contact:
                    self.get_logger().warn('Grasp: segment limit reached without detecting contact')
                self.u_des[:2] = self.u[:2]
                self.phase = 'hold'
                for stage in (0, 1):
                    for i in self.inside(stage, phi):
                        self.squeeze_vec[i] = self.squeeze
            else:
                self.u_des[:2] = min(theta + 2 * step, self.max_segment_bend) * np.array([np.cos(phi), np.sin(phi)])

    def step(self):
        self.t += self.dt
        self.update_target()
        self.u = motions.limit_step(self.u, self.u_des, self.rate * self.dt)
        q = kin.u_to_q(self.u)
        angles = kin.servo_angles(q, self.geo, self.signs) + self.signs * self.squeeze_vec
        if self.active and not self.dry_run:
            self.command_pub.publish(Float64MultiArray(data=[float(a) for a in angles]))

        shown = self.u
        pos = self.state['position']
        if not self.dry_run and pos is not None and len(pos) == 6:
            q_est, self.u_est = kin.estimate_q(pos, self.geo, self.signs, self.u_est)
            self.q_est_pub.publish(Float64MultiArray(data=[float(np.degrees(v)) for v in q_est]))
            shown = self.u_est
        self.publish_joints(kin.u_to_q(shown))
        self.state_pub.publish(String(data=f'{self.mode}:{self.phase}' if self.phase else self.mode))

    def publish_joints(self, q):
        values = []
        for R in kin.joint_rotations(q, self.geo):
            values += [float(v) for v in urdf.euler_xyz(R)]
        msg = JointState(name=self.joint_names, position=values)
        msg.header.stamp = self.get_clock().now().to_msg()
        self.joint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GripperControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
