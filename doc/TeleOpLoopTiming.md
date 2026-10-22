# TeleOp loop timing

Target: an actual start-to-start control period at or below 20 ms (50 Hz).
The changes below remove avoidable overhead. Their effect must be measured on
the Control Hub; desktop tests cannot certify a hardware loop period.

## Changes

- TeleOpRed uses LinearOpMode. In the installed FTC SDK 12 source, iterative
  OpMode performs a telemetry update and `Thread.sleep(1)` after each user loop.
  The new loop performs its own scheduled telemetry updates and uses
  `opModeIsActive()` to yield and observe STOP.
- Status formatting/transmission runs at 5 Hz. Localization, turret PD,
  flywheel velocity control, intake velocity control, and breakbeams continue
  to update every control iteration.
- Flywheel and turret retain the last motor command and skip exact duplicates.
  Small velocity/position feedback corrections still go through unchanged.
- Hood commands smaller than 0.001 servo units accumulate against the last
  command before another write. Commands remain clipped to 0.16–0.485.
- One cached Pedro pose is reused for shooter calculations and telemetry.
- Angular compensation reuses `follower.velocity().omega`; it adds arithmetic
  only, with no extra Pinpoint read. Translation and heading prediction share
  one shooting pose per iteration.
- Automatic shot interpolation reuses one output object instead of allocating
  a shot every iteration. Table validation and invalid-input shutdown remain
  active every iteration. This reduces garbage creation, not hardware I/O.
- Cleanup runs in `finally`, zeros outputs, and restores previous hub cache
  modes on STOP or an exception.

Manual Lynx bulk caching was already enabled and is preserved. The installed
Pedro 3.0.2 drivetrain already caches small drive-power changes and repeated
brake-mode requests. Its Pinpoint update uses one bulk I2C read; the FTC driver
already sets 400 kHz bus speed. Those settings are not the remaining easy fixes.

## Read the measurements

After deploying, let at least 100 loops run, then check stationary, driving,
turret movement, intake, and shooting:

- **Loop avg / max (last 100, ms):** actual periods, including telemetry and
  scheduling between iterations. Check the maximum along with the average.
- **Loops over 20 ms:** count of missed targets in that window.
- **Between-loop gap:** time from completing the previous user iteration to
  starting the next, including yielding and Android scheduling.
- **Control / drive / shooter:** hardware and controller work inside the user
  iteration. Drive includes the Pinpoint update and drive motor commands.
- **Inputs / intake+servos:** bulk cache clearing, beam/input processing and
  drive-command preparation, followed by intake feedback and gate/flower writes.
- **Previous telemetry:** formatting/transmission cost for the previous status
  update; it is included in the measured loop periods.

The prior reported baseline was approximately 28 ms per loop with 12–21 ms
control, 5–7 ms drive, and 0–4 ms shooter work. Compare using the same workload.
If the new control work itself exceeds 20 ms, the section times identify where
additional hardware I/O reduction is needed. A large between-loop gap instead
points toward scheduling or other activity on the Robot Controller.
