# Cart-pole USB control

This branch runs the Wheeltec linear cart-pendulum from a Python controller on a PC. The STM32F103C8 samples the cart and pole every 5 ms and applies the most recent valid signed motor effort. Python owns the control law and can use the entire pole rotation for swing-up and balance.

## Build and connect

1. Open `USER/WHEELTEC.uvprojx` in Keil µVision, select **MiniBalance**, and build with Arm Compiler 6. Install the Keil STM32F1xx device pack if prompted. The flash image is `OBJ/WHEELTEC.hex`. Flash the board with your usual programmer.
2. **Place the cart at the left end before resetting the board.** The incremental encoder starts at 10000 at reset. With the drive disarmed, move the cart by hand toward the middle (around 7925). The firmware permits drive only strictly between 5900 and 9900. Repeat the reference procedure after any reset. The bounds assume the kit's original travel and encoder direction.
3. Connect the kit's micro-USB port to the PC with a data cable and power the motor from the kit supply. The USB-to-serial bridge carries USART1 on PA9/PA10. Use one serial client at a time. [Kit pin map](https://github.com/playfultechnology/pid-invertedpendulum/blob/main/download/pins.txt)

From the repository root, install pySerial and watch the read-only monitor. Substitute your serial device, such as `COM3` on Windows.

```sh
python3 -m pip install -r host/requirements.txt
python3 -m host.cartpole /dev/ttyUSB0 --sensors-only
```

The monitor prints the raw pole angle, cart encoder count, and supply voltage five times per second. Press Ctrl-C to exit. Omit `--sensors-only` to include timing, motor effort, and command status. Move the rod and cart by hand to check the readings. The raw angle spans 0–4095, with about 1024 hanging down and a vendor upright reference of 3100. Encoder counts decrease as the cart travels to the right. [Kit calibration](https://github.com/playfultechnology/pid-invertedpendulum#calibration)

## Python host scripts

Run these modules from the repository root. Each accepts `--help`. Close the current serial client before opening another one.

| Script | Use on the bench | Output |
| --- | --- | --- |
| [host/cartpole.py](host/cartpole.py) | Run `python3 -m host.cartpole /dev/ttyUSB0 --sensors-only` while moving the cart and pole by hand. | Pole angle ADC, cart encoder count, and supply voltage on the terminal. |
| [host/identify.py](host/identify.py) | Collect the experiment runs below. The default mode records with the drive disarmed. Adding `--step-effort` arms the drive for a pair of opposite pulses. | A new CSV containing board timestamps, raw sensors, voltage, experiment phase, requested and applied effort, command acknowledgements, status flags, and missed ticks. |
| [host/pid_controller.py](host/pid_controller.py) | [Balance a hand-raised pole](#run-the-upright-pid) while adjusting the PID gains. | Live measurements and commanded and applied effort on the terminal. |
| [host/example_controller.py](host/example_controller.py) | [Run your own control law](#write-a-controller) by editing `compute_effort`. | Sends the returned effort to the board on each fresh sample. The template returns zero. |

## Hardware experiments

The first simulator fit uses **two experiments** after calibration, one for pendulum free decay and one for cart motor response. Reserve a separate free-pole run to check the fitted model. Give each run a new CSV filename and note the pole configuration, payload and starting position alongside it.

### Calibrate the sensors and weigh the assemblies

- Measure cart travel in metres between several marked positions, logging encoder counts at each, then fit a slope and offset. Choose a positive direction and record it. Repeat after approaching the marks from the other direction to check for backlash.
- Record angle ADC values with the pole hanging, upright, and horizontal on each side. Turn the pole through the sensor wrap and check the sign, usable angular range, and any discontinuity. The Python angle helper uses zero upright. When using equations with zero hanging, convert this convention explicitly. A stationary recording gives a measure of sensor noise.
- Weigh the moving cart and the complete rotating pole assembly separately. Include the pole's attached weights and fittings. Locate the assembled pole's center of mass by balancing it horizontally and measuring from the pivot to the balance point. Record the rail limits, supply voltage, test payload, and mounting arrangement for each dataset.

### Experiment 1 — Pendulum free decay

Hold the cart fixed with the motor disarmed. Release the pole about 5–10° from **hanging down**, with no push. Record several cycles in both directions. Start the logger before release, then repeat at a larger angle for validation.

```sh
python3 -m host.identify /dev/ttyUSB0 pole_decay.csv --duration 20
```

Use the time between same-side angle peaks and the drop in their amplitudes to [estimate pole inertia and pivot damping](#measure-pendulum-inertia-and-pivot-losses).

### Experiment 2 — Cart motor response

Remove the pole or secure it rigidly to the moving carriage, and record which mass moved. Reference the rail as described above and start each run near the middle. Use short positive and negative effort pulses with zero-effort intervals and room for the cart to stop. Repeat from rest at several amplitudes to find the effort where motion starts in each direction. Use a fresh output file per run. The following example requests an 8% pulse and a pulse in the opposite direction. The displacement guard is in encoder counts from the start of the run.

```sh
python3 -m host.identify /dev/ttyUSB0 cart_08.csv --step-effort 0.08 --pulse-seconds 0.10 --travel-counts 250
```

Use the encoder trace and board-reported applied effort to [fit the cart's drive response](#identify-the-cart-drive), including how it accelerates under power and slows at zero effort.

### Validation run with the pole free

Replay an unused effort sequence with the pole free. Compare measured and simulated cart displacement, reversal timing, angular phase and swing height over a few oscillations. Use a different effort level and initial angle from the fitting runs. Larger-amplitude validation matters for swing-up even if the small-angle decay matches.

Use the same `host.identify` pulse command as in experiment 2 with a new filename such as `free_pole_validation.csv`. Set the pulse length or amplitude using what you learned about the cart's travel in experiment 2. Set `--rest-seconds` to several measured pendulum periods to capture the swing after the last pulse. This option also sets the pauses before and between pulses. Keep this CSV out of the parameter fit.

## Run the upright PID

Close the monitor, move the referenced cart near the middle of the track, and hold the pole upright. Start the controller from the repository root, substituting your serial port. On Windows you can use `py -3` in place of `python3`.

```sh
python3 -m host.pid_controller /dev/ttyUSB0
```

The script runs the angle PID on each fresh board sample and holds the cart near its starting position with the kit's filtered position PD correction. It arms only within 500 ADC counts (about 44°) of upright. Leaving that window stops the run. Ctrl-C disarms the drive and exits.

Sensor rows print at 10 Hz by default. Each row includes board time, raw and calibrated pole angle, angular velocity, cart encoder position and velocity, supply voltage, the command just sent, and the effort reported by the board for that sample. The print rate is independent of the control updates.

Use `--kp`, `--ki`, and `--kd` to tune the angle loop in normalized effort, radians, and seconds. The defaults convert the vendor's proportional and derivative gains to these units. Integral gain starts at zero and the integrator accounts for output saturation. These starting gains still need testing with your kit and PC latency. Use `--upright-adc` for your measured upright value, `--max-effort` to limit drive magnitude, and `--print-hz` to set the display rate. The following command checks live readings and commands with zero drive.

```sh
python3 -m host.pid_controller /dev/ttyUSB0 --max-effort 0
```

## Identify the simulator parameters

Start with cart position and velocity, pole angle and angular velocity, and the same signed-effort input as the firmware. Use full nonlinear cart–pole dynamics for swing-up. For a physical pole, the geometry consists of the pivot-to-center-of-mass distance and the moment of inertia about the pivot. The moving cart mass includes the carriage and hardware that travels with it. Rotating transmission parts can add effective inertia to the cart. [Cart–pole equations](https://underactuated.mit.edu/acrobot.html#cart_pole)

For example, choose angle `q = 0` hanging down, with the pole's center of mass at horizontal position `x + l sin(q)`. Let `M` be effective cart-only mass, `m` pole mass, `l` pivot-to-center-of-mass distance, and `J` pole inertia about the pivot. The two equations to solve together at each integration step are

```text
(M + m) x_ddot + m l cos(q) q_ddot - m l sin(q) q_dot² = F_net
J q_ddot + m l cos(q) x_ddot + m g l sin(q) = -b_p q_dot
```

`F_net` includes drive and cart losses. The second equation uses viscous pivot damping `b_p`. For controller-in-the-loop simulation, integrate between 5 ms sample instants, hold the last applied effort between updates, and reproduce the observed command delay and output limits.

### Measure pendulum inertia and pivot losses

Measure the period `T` between same-side peaks. For light damping, the pivot inertia is approximately `J = m g l (T / 2π)²`, where `l` is the measured pivot-to-center-of-mass distance. If the amplitude decays exponentially, fit a straight line to log peak amplitude versus time with slope `−λ`. The damped small-angle estimate is `J = m g l / ((2π/T)² + λ²)`, and the viscous damping is `b = 2 J λ`. Approximately constant amplitude loss per cycle suggests including dry pivot friction. Check that the logged cart position stayed fixed during the release. [Physical pendulum measurement](https://www.rose-hulman.edu/Class/phao/PH/JOW/Ph113%20Labs/Physics_lab_manual/chapter_3/inertia/inertia.html)

### Identify the cart drive

A practical first model while the cart is moving is `Veff = kS sign(v) + kV v + kA a`. Use the measured supply voltage times the board-reported applied effort times `6900/7200` as the voltage proxy, with sign aligned to the chosen cart axis. Fit the breakaway threshold separately near rest. The coefficients include the motor, driver, gearing, friction and the test load. Add a motor torque lag only if a repeatable transient remains after accounting for command timing. Zero-effort deceleration can include electrical braking as well as mechanical friction. [Motor identification model](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/introduction.html)

Fit the model by integrating it from each run's initial state and minimizing position error against the encoder trace. Twice differentiating individual 5 ms encoder samples amplifies quantization noise. Use the unwrapped MCU `elapsed_us` and treat `applied_effort` on row *k* as the input over the **following** interval. The command in `requested_effort` on that row can take effect later. Look for its tick in subsequent `last_command_tick` acknowledgements. The client records the newest available observation and reports gaps in `missed_ticks`. Split or exclude intervals that skip an input transition. The separate PC monotonic timestamp `host_rx_ns` describes arrival jitter on the host clock.

For a force-input cart–pole simulator, the constrained test gives a force gain `K = M_test_eff / kA` in N/V. The moving-cart drive law then becomes `F = K (Veff − kV v − kS sign(v))`. Use the effective mass of the actual test configuration for this conversion. For a test with the pole secured, subtract its mass from the identified effective moving mass when assigning the cart-only mass in the coupled model. If reflected inertia is unknown, repeat the same runs with a known added cart mass `ΔM`. With stable drivetrain characteristics, `K ≈ ΔM / (kA_loaded − kA_base)`. An encoder-only run at one unknown effective mass identifies acceleration per input but leaves that absolute force scale ambiguous.

### Future velocity-loop experiment

The current target applies motor effort directly. If you add a proportional cart-velocity loop, implement the same velocity estimator, gain, sample period, delay and saturation in the simulator. Validate that loop with separate positive and negative velocity-reference steps, logging the reference, measured velocity and applied effort together. Compare rise time, steady-state error, overshoot and travel. For an approximately first-order, unsaturated response, the ratio of settled velocity change to reference change estimates the closed-loop gain. Time from the end of the delay to 63% of that settled change estimates the closed-loop time constant. Those measurements characterize the combined controller and plant.

## Write a controller

Edit `compute_effort(state, motion)` in `host/example_controller.py`. It starts with zero output so you can exercise the command path and observe acknowledgements before introducing a control law. Close the monitor before running the controller.

```sh
python3 -m host.example_controller /dev/ttyUSB0
```

| Python input or output | Meaning |
| --- | --- |
| `state.angle_rad(upright_adc=3100)` | Estimated pole angle in radians, zero upright, wrapped to [−π, π). Positive follows increasing ADC counts. Adjust the upright reference for your kit. |
| `motion.angular_velocity_rad_s` | Angular velocity from the shortest wrapped difference between samples. |
| `state.encoder`, `motion.cart_velocity_counts_s` | Cart position in encoder counts and velocity in counts/s. Measure counts per metre on your kit for SI units. |
| `motion.dt_s` | Time between observations using the board's microsecond clock, including any skipped samples. |
| Return value of `compute_effort` | A finite float from −1 to +1. Magnitude scales linearly to the kit's PWM limit of 6900 out of 7200. The sign selects motor direction. |

The example arms at zero effort, waits for the board's acknowledgement, and calls the controller on each newest complete state. Keep expensive initialization outside the armed loop. The velocity estimates are finite differences. If filtering is needed, its delay is part of the controller design. The board's `tick` and echoed `last_command_tick` let you measure sample-to-application delay in 5 ms steps when a command is first acknowledged.

The USER button acts as a local stop. Pressing it, hitting a rail bound, or dropping below 7.00 V zeros and disarms the drive. The board also disarms when the sample underlying the last accepted arm or effort command reaches 10 ticks of age (50 ms nominal). A fresh arm is required after the condition clears. The board LED lights while armed. The Python context manager sends a stop when an active controller exits, and the firmware timeout covers a lost USB connection.

## Serial protocol

The link uses **115200 baud, 8N1**. Every frame starts with bytes `A5 5A`. All multibyte fields are little-endian. A trailing uint16 CRC-16/CCITT-FALSE covers the entire frame up to the CRC, including sync and type (polynomial 0x1021, initial value 0xFFFF, no reflection, final XOR 0).

| Direction | Fields in transmission order | Size |
| --- | --- | --- |
| STM32 → PC | sync, type `0x12`, flags u8, tick u32, sample time in µs u32, angle ADC u16, encoder i32, supply in centivolts u16, applied effort i16, last accepted ARM/EFFORT sample tick u32, CRC u16 | 28 bytes |
| PC → STM32 | sync, type `0x11`, opcode u8, referenced sample tick u32, effort i16, CRC u16 | 12 bytes |

Wire effort ranges from −1000 to +1000. Opcodes are DISARM (0), ARM (1), and EFFORT (2). ARM and DISARM carry zero effort. A valid DISARM is unconditional. ARM and EFFORT must reference an already sampled tick aged 1–9 ticks, newer than the previous accepted command and inside the current arming window. The board discards damaged, duplicate and stale commands. A valid EFFORT requires an armed board.

Status bit masks are ARMED (0x01), LOCAL_STOP (0x02), RAIL_LIMIT (0x04), LOW_VOLTAGE (0x08), and TIMEOUT (0x10). TIMEOUT clears on a successful arm. The acknowledgement starts at 0xFFFFFFFF and records the most recent accepted ARM or EFFORT even after a stop. Both time and tick wrap modulo 2³². Each telemetry frame describes one timer tick, including the effort chosen on that tick. DMA transfers run independently of the control loop. A busy transmitter skips a frame, and the Python client uses the newest complete observation available.

## Firmware and tests

`USER/host_main.c` owns the timer, sensors, motor and DMA link. `BALANCE/CONTROL/host_control.c` handles arming and command validity. The Keil target uses the USB control loop in place of the kit's on-board PD, swing-up, and DataScope UI. The original control routines are available in `BALANCE/CONTROL/control.c`.

Run the protocol and control tests from the repository root. The embedded C regression uses a native C compiler when available.

```sh
python3 -m unittest discover -s tests -v
```
