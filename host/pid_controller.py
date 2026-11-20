"""Balance a hand-raised pendulum with a PC-side PID and print live measurements."""

import argparse
import math
import time
from concurrent.futures import ThreadPoolExecutor

from .cartpole import CartPole, Motion, State


PWM_LIMIT = 6900
RADIANS_PER_COUNT = math.tau / 4096
# Vendor angle gains are in PWM/count and PWM/(count per 5 ms sample).
DEFAULT_KP = 400 / (PWM_LIMIT * RADIANS_PER_COUNT)
DEFAULT_KD = DEFAULT_KP * 0.005
CAPTURE_ANGLE = 500 * RADIANS_PER_COUNT


class BalancePID:
    """Upright angle PID plus the kit's slower cart-position correction."""

    def __init__(self, initial_state: State, *, kp=DEFAULT_KP, ki=0.0, kd=DEFAULT_KD,
                 max_effort=1.0, upright_adc=3100):
        if not all(math.isfinite(gain) for gain in (kp, ki, kd)):
            raise ValueError("PID gains must be finite")
        if not 0 <= max_effort <= 1:
            raise ValueError("max-effort must be between 0 and 1")
        if not 0 <= upright_adc < 4096:
            raise ValueError("upright-adc must be between 0 and 4095")
        self.kp, self.ki, self.kd = kp, ki, kd
        self.max_effort = max_effort
        self.upright_adc = upright_adc
        self.integral_effort = 0.0
        self.cart_target = initial_state.encoder
        self.cart_tick = initial_state.tick
        self.cart_time_us = initial_state.time_us
        self.cart_error = 0.0
        self.cart_effort = 0.0

    def check_upright(self, state: State) -> float:
        angle = state.angle_rad(self.upright_adc)
        if abs(angle) >= CAPTURE_ANGLE:
            raise RuntimeError(
                f"Pole outside the upright balance window (angle_adc={state.angle_adc}, "
                f"angle_deg={math.degrees(angle):+.1f}). Hold it upright and restart."
            )
        return angle

    def update(self, state: State, motion: Motion) -> float:
        error = -self.check_upright(state)
        if (state.tick - self.cart_tick) % (1 << 32) >= 5:
            dt = ((state.time_us - self.cart_time_us) % (1 << 32)) / 1_000_000
            # At 25 ms the vendor filter retains 80% of the previous error.
            weight = 1 - 0.8 ** (dt / 0.025)
            change = weight * (state.encoder - self.cart_target - self.cart_error)
            self.cart_error += change
            self.cart_effort = (20 * self.cart_error + 300 * 0.025 * change / dt) / PWM_LIMIT
            self.cart_tick, self.cart_time_us = state.tick, state.time_us

        feedback = self.kp * error - self.kd * motion.angular_velocity_rad_s - self.cart_effort
        delta = self.ki * error * motion.dt_s
        limit = self.max_effort
        if not (math.isfinite(feedback) and math.isfinite(delta)):
            raise ValueError("PID gains produced a non-finite control term")
        # Integrate only as far as the remaining actuator range in this direction.
        output = feedback + self.integral_effort
        if delta > 0:
            delta = min(delta, max(0.0, limit - output))
        else:
            delta = max(delta, min(0.0, -limit - output))
        self.integral_effort = max(-limit, min(limit, self.integral_effort + delta))
        return max(-limit, min(limit, feedback + self.integral_effort))


def print_readings(state: State, motion: Motion, upright_adc: int, effort: float):
    print(
        f"tick={state.tick} time_us={state.time_us} angle_adc={state.angle_adc} "
        f"angle_deg={math.degrees(state.angle_rad(upright_adc)):+.2f} "
        f"omega_rad_s={motion.angular_velocity_rad_s:+.3f} encoder={state.encoder} "
        f"cart_counts_s={motion.cart_velocity_counts_s:+.1f} volts={state.voltage_cv / 100:.2f} "
        f"command={effort:+.3f} applied={state.applied_effort:+.3f}",
        flush=True,
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", help="serial device, e.g. /dev/ttyUSB0 or COM3")
    parser.add_argument("--kp", type=float, default=DEFAULT_KP,
                        help=f"proportional gain in effort/rad (default {DEFAULT_KP:.3f})")
    parser.add_argument("--ki", type=float, default=0.0,
                        help="integral gain in effort/(rad s) (default 0)")
    parser.add_argument("--kd", type=float, default=DEFAULT_KD,
                        help=f"derivative gain in effort/(rad/s) (default {DEFAULT_KD:.3f})")
    parser.add_argument("--upright-adc", type=int, default=3100, help="upright ADC count (default 3100)")
    parser.add_argument("--max-effort", type=float, default=1.0, help="output limit from 0 to 1 (default 1)")
    parser.add_argument("--print-hz", type=float, default=10, help="maximum sensor rows per second (default 10)")
    args = parser.parse_args()
    if not math.isfinite(args.print_hz) or args.print_hz <= 0:
        parser.error("--print-hz must be positive and finite")
    try:
        # Close/disarm the serial connection before waiting for outstanding output.
        with ThreadPoolExecutor(max_workers=1) as printer, CartPole(args.port) as cart:
            previous = cart.read_state()
            pid = BalancePID(previous, kp=args.kp, ki=args.ki, kd=args.kd,
                             max_effort=args.max_effort, upright_adc=args.upright_adc)
            pid.check_upright(previous)
            sample = cart.arm(previous)
            pending = None
            next_print = 0.0
            while sample.armed:
                motion = sample.motion_since(previous)
                effort = pid.update(sample, motion)
                cart.set_effort(sample, effort)
                now = time.monotonic()
                if now >= next_print and (pending is None or pending.done()):
                    if pending is not None:
                        pending.result()
                    pending = printer.submit(print_readings, sample, motion, args.upright_adc, effort)
                    next_print = now + 1 / args.print_hz
                previous, sample = sample, cart.read_state()
            raise RuntimeError(f"Board disarmed at tick {sample.tick} with flags {sample.flags!r}")
    except KeyboardInterrupt:
        pass
    except (OSError, RuntimeError, ValueError) as error:
        parser.exit(1, f"{error}\n")


if __name__ == "__main__":
    main()
