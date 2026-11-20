import io
import math
import struct
import unittest
from binascii import crc_hqx
from contextlib import redirect_stderr, redirect_stdout
from dataclasses import replace
from unittest.mock import patch

from host.cartpole import State, Status
from host.pid_controller import BalancePID, main


UPRIGHT = State(Status.ARMED, 100, 500_000, 3100, 8312, 1200, 0.0, 99)


def telemetry(tick, angle_adc, flags, last_command_tick):
    body = struct.pack(
        "<2sBBIIHiHhI", b"\xa5\x5a", 0x12, flags, tick, tick * 5000,
        angle_adc, 7925, 1200, 0, last_command_tick,
    )
    return body + struct.pack("<H", crc_hqx(body, 0xFFFF))


class PIDTests(unittest.TestCase):
    def test_default_angle_terms_match_vendor_sign_and_normalization(self):
        pid = BalancePID(UPRIGHT)
        sample = replace(UPRIGHT, tick=102, time_us=510_000, angle_adc=3104)
        # Four counts over 10 ms gives two counts of D at the nominal 5 ms.
        expected = -(400 * 4 + 400 * 2) / 6900
        self.assertAlmostEqual(pid.update(sample, sample.motion_since(UPRIGHT)), expected)

    def test_integral_uses_board_elapsed_time(self):
        tilted = replace(UPRIGHT, angle_adc=3080)
        pid = BalancePID(tilted, kp=0.0, ki=2.5, kd=0.0)
        first = replace(tilted, tick=101, time_us=505_000)
        second = replace(first, tick=104, time_us=520_000)
        effort = pid.update(first, first.motion_since(tilted))
        increment = pid.update(second, second.motion_since(first)) - effort
        self.assertAlmostEqual(increment, 2.5 * 20 * math.tau / 4096 * 0.015)

    def test_effort_limit_and_recovery_from_saturation(self):
        previous = replace(UPRIGHT, angle_adc=3050)
        pid = BalancePID(previous, kp=3, ki=100, kd=0, max_effort=0.1)
        for _ in range(100):
            sample = replace(previous, tick=previous.tick + 2, time_us=previous.time_us + 10_000)
            self.assertAlmostEqual(pid.update(sample, sample.motion_since(previous)), 0.1)
            previous = sample
        opposite = replace(previous, angle_adc=3150, tick=previous.tick + 1,
                           time_us=previous.time_us + 5000)
        self.assertAlmostEqual(pid.update(opposite, opposite.motion_since(previous)), -0.1)

    def test_position_correction_uses_starting_cart_position(self):
        pid = BalancePID(UPRIGHT)
        previous = UPRIGHT
        for i in range(1, 6):
            sample = replace(UPRIGHT, tick=100 + i, time_us=500_000 + 5000 * i, encoder=8322)
            effort = pid.update(sample, sample.motion_since(previous))
            if i < 5:
                self.assertEqual(effort, 0)
            previous = sample
        self.assertAlmostEqual(effort, -(20 + 300) * 2 / 6900)

    def test_capture_window_respects_calibration_across_adc_rollover(self):
        near_seam = replace(UPRIGHT, angle_adc=2)
        pid = BalancePID(near_seam, upright_adc=4090)
        pid.check_upright(near_seam)
        for delta in (-500, 500):
            edge = replace(near_seam, angle_adc=(4090 + delta) % 4096)
            with self.assertRaises(RuntimeError):
                pid.check_upright(edge)

    @patch("host.cartpole.serial.Serial")
    def test_hanging_pole_prevents_initial_arm(self, serial_port):
        port = serial_port.return_value
        port.in_waiting = 28
        port.read.side_effect = [telemetry(42, 1024, 0, 0xFFFFFFFF)]
        with patch("sys.argv", ["pid_controller", "loopback"]), redirect_stdout(io.StringIO()), \
                redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as stopped:
            main()
        self.assertNotEqual(stopped.exception.code, 0)
        port.write.assert_not_called()

    @patch("host.cartpole.serial.Serial")
    def test_live_cli_prints_measurements_and_disarms_on_fall(self, serial_port):
        port = serial_port.return_value
        port.in_waiting = 28
        port.read.side_effect = [
            telemetry(100, 3100, 0, 0xFFFFFFFF),
            telemetry(101, 3101, Status.ARMED, 100),
            telemetry(102, 3102, Status.ARMED, 101),
            telemetry(103, 1020, Status.ARMED, 102),
        ]
        port.write.side_effect = len
        output = io.StringIO()
        with patch("sys.argv", ["pid_controller", "loopback"]), redirect_stdout(output), \
                redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as stopped:
            main()
        self.assertNotEqual(stopped.exception.code, 0)
        writes = [call.args[0] for call in port.write.call_args_list]
        self.assertEqual([packet[3] for packet in writes], [1, 2, 2, 0])
        self.assertLess(struct.unpack_from("<h", writes[1], 8)[0], 0)
        line = output.getvalue()
        self.assertIn("angle_adc=3101", line)
        self.assertIn("encoder=7925", line)
        self.assertIn("volts=12.00", line)
        self.assertIn("command=-", line)
        port.close.assert_called_once()
