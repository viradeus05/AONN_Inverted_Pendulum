import math
import shutil
import subprocess
import tempfile
import unittest
from binascii import crc_hqx
from dataclasses import replace
from pathlib import Path
from unittest.mock import patch

from host.cartpole import CartPole, State, Status


ROOT = Path(__file__).resolve().parents[1]
# Shared with the firmware test. Includes negative signed fields and a high-bit timestamp.
STATE_FRAME = bytes.fromhex(
    "a5 5a 12 01 78 56 34 12 ff de bc fa 1b 0c c7 cf ff ff "
    "c4 04 06 ff 77 56 34 12 97 a0"
)
EFFORT_FRAME = bytes.fromhex("a5 5a 11 02 78 56 34 12 06 ff 6f e8")
STATE = State(Status.ARMED, 0x12345678, 0xFABCDEFF, 3099, -12345, 1220, -0.25, 0x12345677)


class InterfaceTests(unittest.TestCase):
    @patch("host.cartpole.serial.Serial")
    def test_read_only_stream_recovers_after_noise_and_partial_frame(self, serial_port):
        port = serial_port.return_value
        damaged = bytearray(STATE_FRAME)
        damaged[12] ^= 0x80
        older = bytearray(STATE_FRAME)
        older[4] -= 1
        older[-2:] = crc_hqx(older[:-2], 0xFFFF).to_bytes(2, "little")
        port.in_waiting = 256
        port.read.side_effect = [b"noise" + damaged + older[:2], older[2:] + STATE_FRAME]
        with CartPole("test") as cart:
            self.assertEqual(cart.read_state(), STATE)
        port.write.assert_not_called()
        port.close.assert_called_once()

    @patch("host.cartpole.serial.Serial")
    def test_arm_effort_and_exception_exit_disarms(self, serial_port):
        port = serial_port.return_value
        port.in_waiting = len(STATE_FRAME)
        port.read.return_value = STATE_FRAME
        port.write.side_effect = len
        with self.assertRaisesRegex(RuntimeError, "controller exited"):
            with CartPole("test") as cart:
                with self.assertRaises(RuntimeError):
                    cart.set_effort(STATE, -0.25)
                reply = cart.arm(replace(STATE, tick=STATE.last_command_tick, flags=Status(0)))
                for invalid in (-1.01, 1.01, math.nan, math.inf):
                    with self.assertRaises(ValueError):
                        cart.set_effort(reply, invalid)
                cart.set_effort(reply, -0.25)
                with self.assertRaises(ValueError):
                    cart.set_effort(reply, 0)
                raise RuntimeError("controller exited")
        writes = [call.args[0] for call in port.write.call_args_list]
        self.assertEqual([frame[3] for frame in writes], [1, 2, 0])
        self.assertEqual(writes[1], EFFORT_FRAME)

    def test_motion_handles_sensor_and_time_wrap(self):
        old = replace(STATE, tick=0xFFFFFFFF, time_us=0xFFFFFE00, angle_adc=4094, encoder=9)
        new = replace(STATE, tick=0, time_us=4488, angle_adc=2, encoder=7)
        motion = new.motion_since(old)
        self.assertAlmostEqual(motion.dt_s, 0.005)
        self.assertAlmostEqual(motion.angular_velocity_rad_s, math.tau * 4 / 4096 / 0.005)
        self.assertAlmostEqual(motion.cart_velocity_counts_s, -400.0)
        with self.assertRaises(ValueError):
            old.motion_since(new)

    @unittest.skipUnless(shutil.which("cc"), "Native C compiler required")
    def test_embedded_control_and_protocol(self):
        with tempfile.TemporaryDirectory() as temp:
            prog = Path(temp) / "firmware_host_test"
            subprocess.run(
                ["cc", "-std=c99", "-Wall", "-Wextra", "-Werror", "-O2",
                 "-ISYSTEM/usart", "-IBALANCE/CONTROL", "tests/firmware_host_test.c",
                 "SYSTEM/usart/host_protocol.c", "BALANCE/CONTROL/host_control.c", "-o", prog],
                cwd=ROOT, check=True,
            )
            subprocess.run([prog], check=True)


if __name__ == "__main__":
    unittest.main()
