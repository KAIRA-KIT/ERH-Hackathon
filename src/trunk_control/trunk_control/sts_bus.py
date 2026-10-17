"""Minimal Feetech STS (ST3215) serial bus driver."""
import threading

import serial

INST_READ = 0x02
INST_WRITE = 0x03
INST_SYNC_WRITE = 0x83
BROADCAST_ID = 0xFE

REG_MODE = 33
REG_TORQUE_ENABLE = 40
REG_ACC = 41
REG_GOAL_POSITION = 42
REG_LOCK = 55
REG_PRESENT_POSITION = 56

MODE_STEP = 3
STEPS_PER_REV = 4096
MAX_STEP_MOVE = 32767


def _checksum(data):
    return (~sum(data)) & 0xFF


def _signed(value, sign_bit):
    """Decode a sign-magnitude value where `sign_bit` is the sign."""
    magnitude = value & ((1 << sign_bit) - 1)
    return -magnitude if value & (1 << sign_bit) else magnitude


class StsBus:
    def __init__(self, port, baudrate=1000000, timeout=0.02):
        self._serial = serial.Serial(port, baudrate, timeout=timeout)
        self._lock = threading.Lock()

    def close(self):
        self._serial.close()

    def _packet(self, servo_id, instruction, params):
        body = [servo_id, len(params) + 2, instruction, *params]
        return bytes([0xFF, 0xFF, *body, _checksum(body)])

    def _read_reply(self, servo_id):
        """Return the reply parameters, or None on timeout/bad packet."""
        header = self._serial.read(4)
        if len(header) < 4 or header[:2] != b'\xff\xff' or header[2] != servo_id:
            return None
        rest = self._serial.read(header[3])
        if len(rest) < header[3]:
            return None
        if _checksum(list(header[2:]) + list(rest[:-1])) != rest[-1]:
            return None
        return list(rest[1:-1])  # drop error byte and checksum

    def write(self, servo_id, address, data):
        with self._lock:
            self._serial.reset_input_buffer()
            self._serial.write(self._packet(servo_id, INST_WRITE, [address, *data]))
            return self._read_reply(servo_id) is not None

    def read(self, servo_id, address, length):
        with self._lock:
            self._serial.reset_input_buffer()
            self._serial.write(self._packet(servo_id, INST_READ, [address, length]))
            reply = self._read_reply(servo_id)
        if reply is None or len(reply) != length:
            return None
        return reply

    def read_status(self, servo_id):
        """Return (raw_position, speed_steps_s, load_pct, voltage_v) or None."""
        data = self.read(servo_id, REG_PRESENT_POSITION, 7)
        if data is None:
            return None
        position = data[0] | data[1] << 8
        speed = _signed(data[2] | data[3] << 8, 15)
        load = _signed(data[4] | data[5] << 8, 10) * 0.1
        return position, speed, load, data[6] * 0.1

    def set_step_mode(self, servo_id):
        ok = self.write(servo_id, REG_LOCK, [0])
        ok &= self.write(servo_id, REG_MODE, [MODE_STEP])
        ok &= self.write(servo_id, REG_LOCK, [1])
        return ok

    def torque(self, servo_id, enable):
        return self.write(servo_id, REG_TORQUE_ENABLE, [1 if enable else 0])

    def sync_move_relative(self, steps_by_id, speed, acc):
        """Command each servo to move by a signed number of steps (step mode)."""
        block = []
        for servo_id, steps in steps_by_id.items():
            steps = max(-MAX_STEP_MOVE, min(MAX_STEP_MOVE, int(steps)))
            raw = abs(steps) | (0x8000 if steps < 0 else 0)
            block += [servo_id, acc, raw & 0xFF, raw >> 8, 0, 0, speed & 0xFF, speed >> 8]
        # per servo: acc(41), position(42-43), time(44-45), speed(46-47) = 7 bytes
        params = [REG_ACC, 7, *block]
        with self._lock:
            self._serial.write(self._packet(BROADCAST_ID, INST_SYNC_WRITE, params))
