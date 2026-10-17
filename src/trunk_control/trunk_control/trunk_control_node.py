import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Float64MultiArray

from trunk_control.sts_bus import STEPS_PER_REV, StsBus

DEG_PER_STEP = 360.0 / STEPS_PER_REV


class TrunkControlNode(Node):
    def __init__(self):
        super().__init__('trunk_control')
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 1000000)
        self.declare_parameter('servo_ids', [1, 2, 3, 4, 5, 6])
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('goal_speed', 2000)
        self.declare_parameter('goal_acc', 50)
        self.declare_parameter('configure_step_mode', True)

        self.ids = list(self.get_parameter('servo_ids').value)
        self.goal_speed = int(self.get_parameter('goal_speed').value)
        self.goal_acc = int(self.get_parameter('goal_acc').value)
        n = len(self.ids)

        self.bus = StsBus(self.get_parameter('port').value,
                          self.get_parameter('baudrate').value)
        for servo_id in self.ids:
            if self.get_parameter('configure_step_mode').value:
                if not self.bus.set_step_mode(servo_id):
                    self.get_logger().warn(f'Servo {servo_id}: step mode config failed')
            if not self.bus.torque(servo_id, True):
                self.get_logger().warn(f'Servo {servo_id}: no response')

        # Multi-turn tracking: continuous steps since start, and the zero reference.
        self.last_raw = [None] * n
        self.cont_steps = [0] * n
        self.zero_steps = [0] * n

        self.pubs = {
            name: self.create_publisher(Float64MultiArray, '~/' + name, 10)
            for name in ('position', 'load', 'speed', 'voltage')
        }
        self.create_subscription(Float64MultiArray, '~/command', self.on_command, 10)
        self.create_subscription(Empty, '~/zero', self.on_zero, 10)
        self.create_timer(1.0 / self.get_parameter('rate_hz').value, self.poll)

    def poll(self):
        n = len(self.ids)
        nan = math.nan
        out = {name: [nan] * n for name in self.pubs}
        for i, servo_id in enumerate(self.ids):
            status = self.bus.read_status(servo_id)
            if status is None:
                self.get_logger().warn(f'Servo {servo_id}: read failed',
                                       throttle_duration_sec=2.0)
                continue
            raw, speed, load, voltage = status
            if self.last_raw[i] is not None:
                d = raw - self.last_raw[i]
                if d > STEPS_PER_REV // 2:
                    d -= STEPS_PER_REV
                elif d < -STEPS_PER_REV // 2:
                    d += STEPS_PER_REV
                self.cont_steps[i] += d
            self.last_raw[i] = raw
            out['position'][i] = (self.cont_steps[i] - self.zero_steps[i]) * DEG_PER_STEP
            out['speed'][i] = speed * DEG_PER_STEP
            out['load'][i] = load
            out['voltage'][i] = voltage
        for name, values in out.items():
            self.pubs[name].publish(Float64MultiArray(data=values))

    def on_command(self, msg):
        if len(msg.data) != len(self.ids):
            self.get_logger().warn(
                f'Command needs {len(self.ids)} values, got {len(msg.data)}; ignored')
            return
        moves = {}
        for i, servo_id in enumerate(self.ids):
            if self.last_raw[i] is None:
                continue
            target = round(msg.data[i] / DEG_PER_STEP)
            moves[servo_id] = target - (self.cont_steps[i] - self.zero_steps[i])
        if moves:
            self.bus.sync_move_relative(moves, self.goal_speed, self.goal_acc)

    def on_zero(self, _msg):
        self.zero_steps = list(self.cont_steps)
        self.get_logger().info('Zeroed all servos')

    def destroy_node(self):
        self.bus.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TrunkControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
