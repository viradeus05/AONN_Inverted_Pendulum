"""USB serial interface for the Wheeltec linear cart-pendulum."""

import argparse
import math
import struct
import time
from binascii import crc_hqx
from dataclasses import dataclass
from enum import IntFlag
from typing import Iterator

import serial


_SYNC = b"\xa5\x5a"
_STATE_HEADER = _SYNC + b"\x12"
_STATE = struct.Struct("<2sBBIIHiHhI")
_COMMAND = struct.Struct("<2sBBIh")
_CRC = struct.Struct("<H")
_MASK32 = 0xFFFFFFFF


class Status(IntFlag):
    ARMED = 1
    LOCAL_STOP = 2
    RAIL_LIMIT = 4
    LOW_VOLTAGE = 8
    TIMEOUT = 16


@dataclass(frozen=True)
class Motion:
    dt_s: float
    angular_velocity_rad_s: float
    cart_velocity_counts_s: float


def _angle_delta_rad(current: int, reference: int) -> float:
    counts = (current - reference + 2048) % 4096 - 2048
    return counts * (math.tau / 4096)


@dataclass(frozen=True)
class State:
    flags: Status
    tick: int
    time_us: int
    angle_adc: int
    encoder: int
    voltage_cv: int
    applied_effort: float
    last_command_tick: int

    @property
    def armed(self) -> bool:
        return bool(self.flags & Status.ARMED)

    def angle_rad(self, upright_adc: int = 3100) -> float:
        """Angle in [-pi, pi), with zero upright and positive along increasing ADC."""
        return _angle_delta_rad(self.angle_adc, upright_adc)

    def motion_since(self, previous: "State") -> Motion:
        """Raw finite differences using the shortest angular arc and MCU time.

        Accept gaps up to ten ticks and 50 ms. Restart the estimate after a
        longer gap or a board reset. Calibrate counts per metre for SI cart speed.
        """
        ticks = (self.tick - previous.tick) & _MASK32
        delta_us = (self.time_us - previous.time_us) & _MASK32
        if not (1 <= ticks <= 10 and 0 < delta_us <= 50_000):
            raise ValueError("Samples must be ordered and at most 50 ms apart")
        dt_s = delta_us / 1_000_000
        return Motion(
            dt_s,
            _angle_delta_rad(self.angle_adc, previous.angle_adc) / dt_s,
            (self.encoder - previous.encoder) / dt_s,
        )


class CartPole:
    """Read measurements and submit one normalized motor command per sample."""

    def __init__(self, port: str, timeout: float = 0.5):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("timeout must be positive and finite")
        self._timeout = timeout
        # Leave the bridge's optional boot/reset control lines idle on open.
        self._port = serial.Serial(
            port=None, baudrate=115200, timeout=min(timeout, 0.01), write_timeout=timeout,
            exclusive=True,
        )
        self._port.dtr = False
        self._port.rts = False
        self._port.port = port
        self._port.open()
        self._buffer = bytearray()
        self._last_sent_tick = None
        self._arm_requested = False
        self._arm_confirmed = False

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, traceback):
        self.close()

    def close(self):
        """Release the port. A session that requested control sends a final stop."""
        try:
            if self._arm_requested:
                self._best_effort_disarm()
        finally:
            self._port.close()

    def _latest_frame(self):
        frame_size = _STATE.size + _CRC.size
        data = self._buffer
        pos = 0
        latest = None
        while True:
            start = data.find(_STATE_HEADER, pos)
            if start < 0:
                # A partial header can straddle reads.
                pos = max(pos, len(data) - len(_STATE_HEADER) + 1)
                break
            if len(data) - start < frame_size:
                pos = start
                break
            end = start + _STATE.size
            expected_crc = _CRC.unpack_from(data, end)[0]
            if crc_hqx(data[start:end], 0xFFFF) != expected_crc:
                pos = start + 1
                continue
            latest = _STATE.unpack_from(data, start)
            pos = start + frame_size

        del data[:pos]
        if latest is not None:
            _, _, flags, tick, time_us, angle, encoder, voltage, effort, last = latest
            return State(Status(flags), tick, time_us, angle, encoder, voltage, effort / 1000, last)
        return None

    def _read_until(self, deadline: float) -> State:
        while time.monotonic() < deadline:
            # Read one snapshot of queued bytes. Return its newest intact sample.
            self._buffer.extend(self._port.read(self._port.in_waiting or 1))
            state = self._latest_frame()
            if state is not None:
                if not state.armed:
                    self._arm_confirmed = False
                return state
        raise TimeoutError("No valid cart-pendulum state before the serial deadline")

    def read_state(self) -> State:
        """Return the newest complete sample available, waiting up to the timeout."""
        return self._read_until(time.monotonic() + self._timeout)

    def states(self) -> Iterator[State]:
        """Yield fresh telemetry until the port times out or the caller exits."""
        while True:
            yield self.read_state()

    def _send(self, opcode: int, tick: int, effort: int):
        body = _COMMAND.pack(_SYNC, 0x11, opcode, tick, effort)
        frame = body + _CRC.pack(crc_hqx(body, 0xFFFF))
        if self._port.write(frame) != len(frame):
            raise serial.SerialTimeoutException("Incomplete cart-pendulum command")

    def arm(self, state: State) -> State:
        """Request zero effort and return the sample acknowledging this arm request."""
        if state.armed:
            raise RuntimeError("The board is already armed")
        if state.flags & (Status.LOCAL_STOP | Status.RAIL_LIMIT | Status.LOW_VOLTAGE):
            raise RuntimeError(f"Board interlock prevents arming (flags {state.flags!r})")
        self._arm_requested = True
        self._arm_confirmed = False
        try:
            self._send(1, state.tick, 0)
            deadline = time.monotonic() + self._timeout
            while time.monotonic() < deadline:
                reply = self._read_until(deadline)
                if reply.armed and reply.last_command_tick == state.tick:
                    self._last_sent_tick = state.tick
                    self._arm_confirmed = True
                    return reply
                age_ticks = (reply.tick - state.tick) & _MASK32
                if 10 <= age_ticks < 0x80000000:
                    raise RuntimeError(f"Arm request expired with board flags {reply.flags!r}")
            raise TimeoutError("Timed out waiting for arm acknowledgement")
        finally:
            if not self._arm_confirmed:
                self._best_effort_disarm()

    def set_effort(self, state: State, effort: float):
        """Drive in [-1, 1] using a newer sample obtained after arm acknowledgement."""
        if not self._arm_confirmed or not state.armed:
            raise RuntimeError(f"An armed session is required (board flags {state.flags!r})")
        if not math.isfinite(effort) or not -1 <= effort <= 1:
            raise ValueError("effort must be finite and in [-1, 1]")
        tick_delta = (state.tick - self._last_sent_tick) & _MASK32
        if not 0 < tick_delta < 0x80000000:
            raise ValueError("Each effort command needs a newer state tick")
        self._send(2, state.tick, round(effort * 1000))
        self._last_sent_tick = state.tick

    def disarm(self):
        """Send an unconditional stop, including from a newly opened session."""
        self._arm_confirmed = False
        self._send(0, 0, 0)
        self._arm_requested = False

    def _best_effort_disarm(self):
        try:
            self.disarm()
        except OSError:
            # An unplugged board stops locally when its command watchdog expires.
            pass


def main():
    parser = argparse.ArgumentParser(description="Monitor cart-pendulum USB telemetry")
    parser.add_argument("port", help="serial device, e.g. /dev/ttyUSB0 or COM3")
    parser.add_argument("--sensors-only", action="store_true",
                        help="print just the pole angle ADC, cart encoder count, and supply voltage")
    args = parser.parse_args()
    next_print = 0.0
    try:
        with CartPole(args.port) as cart:
            for sample in cart.states():
                now = time.monotonic()
                if now >= next_print:
                    sensors = (f"angle={sample.angle_adc} encoder={sample.encoder} "
                               f"volts={sample.voltage_cv / 100:.2f}")
                    if args.sensors_only:
                        print(sensors, flush=True)
                    else:
                        print(f"tick={sample.tick} us={sample.time_us} {sensors} "
                              f"effort={sample.applied_effort:+.3f} ack={sample.last_command_tick} "
                              f"flags={sample.flags!r}", flush=True)
                    next_print = now + 0.2
    except KeyboardInterrupt:
        pass
    except OSError as error:
        parser.exit(1, f"{error}\n")


if __name__ == "__main__":
    main()
