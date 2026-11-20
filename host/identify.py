"""Log cart-pole telemetry to CSV for calibration and system identification."""

import argparse
import csv
import math
from pathlib import Path
import time

from .cartpole import CartPole


COLUMNS = (
    "host_rx_ns", "tick", "time_us", "elapsed_us", "missed_ticks", "angle_adc",
    "encoder", "voltage_cv", "flags", "applied_effort", "last_command_tick",
    "phase", "requested_effort",
)


def write_sample(writer, state, rx_ns, elapsed_us, missed, phase, requested=None):
    # This row's applied effort was selected on board at this tick. A requested
    # effort is a subsequent host command referencing this row's sample.
    writer.writerow((
        rx_ns, state.tick, state.time_us, elapsed_us, missed, state.angle_adc,
        state.encoder, state.voltage_cv, int(state.flags), state.applied_effort,
        state.last_command_tick, phase, requested,
    ))


def step_profile(elapsed_us, effort, pulse_us, rest_us):
    for phase, duration, request in (
        ("baseline", rest_us, 0.0),
        ("step", pulse_us, effort),
        ("coast", rest_us, 0.0),
        ("reverse", pulse_us, -effort),
        ("settle", rest_us, 0.0),
    ):
        if elapsed_us < duration:
            return phase, request
        elapsed_us -= duration
    return "complete", None


def record(cart, writer, duration_us, step_effort, pulse_us, rest_us, travel_counts):
    writer.writerow(COLUMNS)
    state = cart.read_state()
    rx_ns = time.monotonic_ns()
    origin = state.encoder
    previous = None
    elapsed_us = 0
    profile_start_us = None
    powered = step_effort is not None
    if powered:
        write_sample(writer, state, rx_ns, 0, 0, "arm")
        previous = state
        state = cart.arm(state)
        rx_ns = time.monotonic_ns()

    while True:
        missed = 0
        dt_us = 0
        if previous is not None:
            ticks = (state.tick - previous.tick) & 0xFFFFFFFF
            dt_us = (state.time_us - previous.time_us) & 0xFFFFFFFF
            if not (0 < ticks < 0x80000000 and 0 < dt_us < 0x80000000):
                raise RuntimeError("Board clock or tick restarted, repeated or went backwards")
            elapsed_us += dt_us
            missed = ticks - 1

        if powered:
            if profile_start_us is None:
                profile_start_us = elapsed_us
            error = None
            if not state.armed:
                error = f"Board disarmed (flags {state.flags!r})"
            elif abs(state.encoder - origin) >= travel_counts:
                error = f"Travel reached the {travel_counts}-count experiment limit"
            elif missed >= 9 or dt_us >= 50_000:
                error = "Lost 50 ms or more between observations during the motor run"
            if error:
                try:
                    cart.disarm()
                finally:
                    write_sample(writer, state, rx_ns, elapsed_us, missed, "stopped")
                raise RuntimeError(error)
            phase, requested = step_profile(elapsed_us - profile_start_us, step_effort,
                                            pulse_us, rest_us)
            if requested is None:
                cart.disarm()
            else:
                cart.set_effort(state, requested)
            write_sample(writer, state, rx_ns, elapsed_us, missed, phase, requested)
            if requested is None:
                return
        else:
            write_sample(writer, state, rx_ns, elapsed_us, missed, "passive")
            if state.armed:
                raise RuntimeError("Passive logging requires a disarmed board")
            if elapsed_us >= duration_us:
                return

        previous = state
        state = cart.read_state()
        rx_ns = time.monotonic_ns()


def microseconds(value):
    us = float(value) * 1_000_000
    if not math.isfinite(us) or us < 5000:
        raise argparse.ArgumentTypeError("enter a finite duration of at least 0.005 seconds")
    return round(us)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", help="serial device, e.g. /dev/ttyUSB0 or COM3")
    parser.add_argument("output", type=Path, help="new CSV file")
    parser.add_argument("--duration", type=microseconds, metavar="SECONDS",
                        help="passive capture duration (default 15 seconds)")
    parser.add_argument("--step-effort", type=float, metavar="U",
                        help="arm and apply a pulse at U then at -U, normalized to [-1, 1]")
    parser.add_argument("--pulse-seconds", type=microseconds, default=150_000, metavar="S",
                        help="duration of each powered pulse (default 0.15 seconds)")
    parser.add_argument("--rest-seconds", type=microseconds, default=500_000, metavar="S",
                        help="zero-effort time before, between and after pulses (default 0.5 seconds)")
    parser.add_argument("--travel-counts", type=int, default=400,
                        help="stop at this absolute encoder displacement from start (default 400)")
    args = parser.parse_args(argv)
    if args.step_effort is not None:
        if not 0.001 <= abs(args.step_effort) <= 1:
            parser.error("--step-effort magnitude must be between 0.001 and 1")
        if args.duration is not None:
            parser.error("--duration applies to passive capture; set --pulse-seconds and --rest-seconds for motor runs")
    if args.travel_counts <= 0:
        parser.error("--travel-counts must be positive")
    try:
        with args.output.open("x", newline="", encoding="utf-8") as stream, CartPole(args.port) as cart:
            record(cart, csv.writer(stream), args.duration or 15_000_000, args.step_effort,
                   args.pulse_seconds, args.rest_seconds, args.travel_counts)
    except KeyboardInterrupt:
        pass
    except (OSError, ValueError, RuntimeError) as error:
        parser.exit(1, f"{error}\n")


if __name__ == "__main__":
    main()
