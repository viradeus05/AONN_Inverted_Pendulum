import csv
import io
import struct
import tempfile
import unittest
from binascii import crc_hqx
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path
from unittest.mock import patch

from host.cartpole import Status
from host.identify import main


class SerialBoard:
    """A 5 ms sample clock with one-tick effort acknowledgement."""

    def __init__(self):
        self.tick = 0
        self.time_us = 0
        self.flags = Status(0)
        self.encoder = 7925
        self.effort = 0
        self.ack = 0xFFFFFFFF
        self.commands = []
        self.before_sample = lambda: None

    def read(self, size):
        self.before_sample()
        self.tick = (self.tick + 1) & 0xFFFFFFFF
        self.time_us = (self.time_us + 5000) & 0xFFFFFFFF
        body = struct.pack(
            "<2sBBIIHiHhI", b"\xa5\x5a", 0x12, self.flags, self.tick, self.time_us,
            1025, self.encoder, 1210, self.effort, self.ack,
        )
        return body + crc_hqx(body, 0xFFFF).to_bytes(2, "little")

    def write(self, frame):
        _, _, opcode, tick, effort = struct.unpack("<2sBBIh", frame[:-2])
        self.commands.append((opcode, tick, effort))
        if opcode == 0:
            self.flags &= ~Status.ARMED
            self.effort = 0
        else:
            if opcode == 1:
                self.flags |= Status.ARMED
            self.ack, self.effort = tick, effort
        return len(frame)


class IdentifyTests(unittest.TestCase):
    def setUp(self):
        self.board = SerialBoard()
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        self.output = Path(folder.name) / "run.csv"

    def run_logger(self, *options):
        with patch("host.cartpole.serial.Serial") as serial, \
                redirect_stderr(io.StringIO()), redirect_stdout(io.StringIO()):
            port = serial.return_value
            port.in_waiting = 28
            port.read.side_effect = self.board.read
            port.write.side_effect = self.board.write
            main(["loopback", str(self.output), *options])

    def rows(self):
        with self.output.open(newline="") as handle:
            return list(csv.DictReader(handle))

    def test_passive_records_rollover_and_a_gap_without_sending_commands(self):
        board = self.board
        board.tick = 0xFFFFFFFD
        board.time_us = 0xFFFFDA00

        def skip_two_ticks():
            if board.tick == 0xFFFFFFFE:
                board.tick = (board.tick + 2) & 0xFFFFFFFF
                board.time_us = (board.time_us + 10_000) & 0xFFFFFFFF

        board.before_sample = skip_two_ticks
        self.run_logger("--duration", ".025")
        rows = self.rows()
        self.assertEqual(board.commands, [])
        self.assertEqual([int(row["tick"]) for row in rows[:2]], [0xFFFFFFFE, 1])
        self.assertEqual(int(rows[1]["missed_ticks"]), 2)
        self.assertEqual(int(rows[1]["elapsed_us"]), 15000)
        self.assertLess(int(rows[1]["time_us"]), int(rows[0]["time_us"]))
        for row in rows:
            self.assertEqual(int(row["angle_adc"]), 1025)
            self.assertEqual(int(row["encoder"]), 7925)
            self.assertEqual(int(row["voltage_cv"]), 1210)
            self.assertEqual(row["requested_effort"], "")
            self.assertEqual(float(row["applied_effort"]), 0)

    def test_motor_steps_and_csv_align_with_board_acknowledgements(self):
        self.run_logger("--step-effort", "-0.125", "--pulse-seconds", ".010",
                        "--rest-seconds", ".015")
        rows = self.rows()
        commands = self.board.commands
        self.assertEqual([command[0] for command in commands if command[0] != 2], [1, 0])
        self.assertEqual(commands[-1][0], 0)
        sent = {tick: effort for opcode, tick, effort in commands if opcode == 2}
        requested = {int(row["tick"]): round(1000 * float(row["requested_effort"]))
                     for row in rows if row["requested_effort"] != ""}
        self.assertEqual(sent, requested)
        self.assertEqual(list(sent.values()).count(-125), 2)
        self.assertEqual(list(sent.values()).count(125), 2)
        phases = list(dict.fromkeys(row["phase"] for row in rows))
        self.assertEqual(phases, ["arm", "baseline", "step", "coast", "reverse", "settle", "complete"])
        for row in rows:
            ack = int(row["last_command_tick"])
            if ack in sent:
                self.assertEqual(int(row["tick"]), ack + 1)
                self.assertAlmostEqual(float(row["applied_effort"]), sent[ack] / 1000)

    def test_exceeding_travel_guard_records_last_sample_and_disarms(self):
        board = self.board

        def move_past_limit():
            if board.effort:
                board.encoder = 8100

        board.before_sample = move_past_limit
        with self.assertRaises(SystemExit):
            self.run_logger("--step-effort", ".1", "--pulse-seconds", ".05",
                            "--rest-seconds", ".01", "--travel-counts", "100")
        self.assertEqual(board.commands[-1][0], 0)
        last = self.rows()[-1]
        self.assertEqual(int(last["encoder"]), 8100)
        self.assertEqual(last["requested_effort"], "")
        self.assertAlmostEqual(float(last["applied_effort"]), 0.1)

    def test_disconnect_after_a_step_attempts_stop_and_preserves_data(self):
        board = self.board

        def disconnect():
            if board.effort:
                raise OSError("Disconnected")

        board.before_sample = disconnect
        with self.assertRaises(SystemExit):
            self.run_logger("--step-effort", ".1", "--rest-seconds", ".01")
        self.assertEqual(board.commands[-1][0], 0)
        self.assertGreater(float(self.rows()[-1]["requested_effort"]), 0)

    def test_board_interlock_ends_experiment_before_another_effort(self):
        board = self.board

        def stop_button():
            if board.effort:
                board.flags = Status.LOCAL_STOP
                board.effort = 0

        board.before_sample = stop_button
        with self.assertRaises(SystemExit):
            self.run_logger("--step-effort", ".1", "--rest-seconds", ".01")
        self.assertEqual(board.commands[-1][0], 0)
        last = self.rows()[-1]
        self.assertEqual(int(last["flags"]), Status.LOCAL_STOP)
        self.assertEqual(last["requested_effort"], "")


if __name__ == "__main__":
    unittest.main()
