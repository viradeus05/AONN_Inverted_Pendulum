"""Run an editable cart-pole policy once per fresh telemetry sample."""

import argparse

from .cartpole import CartPole, Motion, State


def compute_effort(state: State, motion: Motion) -> float:
    """Replace this function with your policy. Return a value in [-1, 1].

    state.angle_rad() and motion.angular_velocity_rad_s describe the pole.
    state.encoder and motion.cart_velocity_counts_s describe the cart.
    motion.dt_s is the measured interval between these observations.
    """
    return 0.0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", help="serial device, e.g. /dev/ttyUSB0 or COM3")
    args = parser.parse_args()
    try:
        with CartPole(args.port) as cart:
            previous = cart.read_state()
            sample = cart.arm(previous)
            while sample.armed:
                effort = compute_effort(sample, sample.motion_since(previous))
                cart.set_effort(sample, effort)
                previous, sample = sample, cart.read_state()
            raise RuntimeError(f"Board disarmed at tick {sample.tick} with flags {sample.flags!r}")
    except KeyboardInterrupt:
        pass
    except (OSError, RuntimeError, ValueError) as error:
        parser.exit(1, f"{error}\n")


if __name__ == "__main__":
    main()
