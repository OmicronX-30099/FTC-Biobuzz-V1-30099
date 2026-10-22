# Turret Auto Tuner

Select **Turret Auto Tuner** in the **Tests** group. This is a standalone OpMode;
it does not call or modify `TeleOpMode`, and it never writes `TurretConfig`.
It uses the existing turret controller, encoder tracker, hardware calibration,
power cap, friction ramp and deadband. It requires the same configured hardware
as TeleOp, including the drivetrain, Pinpoint, flywheel, servos and breakbeams.

The tuner runs the follower, flywheel, intake, flowers, gate and breakbeam code
with TeleOp's controls and manual hub bulk caching. This reproduces the important
hardware workload while measuring turret response; its diagnostic telemetry adds
some overhead. Start-to-start loop timing is included in the measurements.

## Run a tune

1. Start near forward, or set `TurretConfig.INITIAL_TURRET_ESTIMATE_DEG` within
   40 degrees of the physical starting angle before INIT. The absolute encoder
   repeats every 80 turret degrees. Verify calibration and feedback directions
   with the existing Turret Test first.
2. INIT and START. **START also runs the flywheel at
   `Flywheel.TARGET_VELOCITY_TPS`, just like TeleOp.** Drive, intake, flower and
   shooting controls are active. The turret stays unpowered until A.
3. Keep the robot still for the initial comparison. Press gamepad 1 **A** once.
   The current angle becomes the sweep center. Leave clearance for the sweep
   and overshoot; default targets range from center minus 15 to center plus
   15 degrees. The entire envelope must have another 8 degrees of clearance
   inside the configured travel limits.
4. Wait for **COMPLETE**. At default settings, the search plus validation takes
   roughly 75–100 seconds. It measures the original gains and searches P, D,
   and friction compensation S in two bounded passes. Each trial includes
   three steps and a sinusoidal moving target. Each candidate must settle all
   three steps; the score penalizes mean error, final error and overshoot.
   The winner must pass a separate repeat trial.
5. Press **X** to aim at the field goal using the winning gains, or **Y** to aim
   using the original gains. Drive and rotate to compare responsiveness with
   real pose updates. Both previews use the same target calculation and limits
   as TeleOp. Circle resets the starting pose, as in TeleOp.
6. Press **B** to cancel a sweep or stop turret preview; **A** starts a fresh
   search. Driver Station **STOP** stops all motors and restores hub cache modes.

X/Y comparisons run only after a validated result. B stops the turret; the other
TeleOp controls and flywheel continue until STOP.

## Settings and results

Configure `TurretAutoTune.kt` in source: `SWEEP_AMPLITUDE_DEG`, `STEP_SECONDS`,
`SETTLE_TOLERANCE_DEG`, and `MAX_LOOP_GAP_MS`. Settings are captured at A;
changing controller, calibration or tuning settings during a run stops it.
Restart INIT after calibration changes. The search keeps `MAX_POWER`, deadband
and friction ramp fixed. P is bounded to 0.001–0.2, D to 0–0.02, and S to 0–0.3.

The result appears on Driver Station and Dashboard and is saved to the robot's
`FIRST/settings/turret-auto-tune.txt`. **No gains are applied automatically.**
After comparing the previews, copy the reported P/D/S values into
`Subsystem/Turret/TurretConfig.java` and rebuild/reinstall to use them in TeleOp.
Edits under `TurretTest` affect only that test's separate gain fields.

If no candidate settles, try a longer `STEP_SECONDS` (up to 2 seconds), or check
friction, encoder readings and the power cap. Longer trials may hit the tuner's
150-second timeout. An envelope violation, invalid feedback or a loop gap over
`MAX_LOOP_GAP_MS` aborts tuning and zeros turret power. A gap fault is evidence
to investigate scheduling/hardware latency before changing gains.

This is a local search around the current controller, with measured validation;
it cannot guarantee globally optimal gains. Consistent battery/load conditions
and the X/Y driving comparison matter. If output is already at `MAX_POWER`, gain
changes cannot increase that cap. If the field target moves incorrectly while
turning, inspect localization and heading/angle conventions separately.
