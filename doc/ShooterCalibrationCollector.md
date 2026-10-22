# Shooter Calibration Collector

Select **Shooter Calibration Collector** in the **Experimental** group. This
OpMode uses manual hood/TPS settings and Pedro's Pinpoint localization, without
the physics LUT or its height-feasibility gate. The turret automatically aims at
the selected goal using the existing tuned controller, encoders, and travel limits.
It remains stopped during INIT, begins tracking after START, and stops during
cleanup. It uses the measured pose, without SOTM lead for these stationary shots.
The collector does not command the drivetrain. Production TeleOp and Physics
Shooter Test retain their controls.
Dashboard configuration remains disabled.

## Before START

Place the robot at the normal known start **(8.2,7.3) inches, heading 90 degrees**
before INIT. Move it from there to the shooting position so Pinpoint tracks the
correct coordinates. The turret tracks the selected goal automatically. Goal 2
is selected above measured Y=70.75 inches; otherwise goal 1. The starting hood
setting is **0.300** and the motor target is **1480 TPS**. Shooter/feed motors
remain off until enabled; turret tracking starts automatically after START.

At the reported robot position (59.35,7.68), distance to goal 1 should be about
51.43 inches. Check the displayed pose and distance before recording attempts.
Circle resets coordinates to the known starting pose, so only use it when the
robot is physically back at that start.

## Controls

| PlayStation control | Action |
|---|---|
| Cross | Toggle shooter on/off |
| D-pad up/down | Add/subtract 20 motor TPS per press |
| D-pad right/left | Add/subtract 0.005 hood servo position per press |
| Right bumper | Feed while held; release after **one ball** |
| Square, after releasing feed | Mark latest attempt **SCORE** |
| Triangle, after releasing feed | Mark latest attempt **MISS** |
| Left bumper | Stop shooter/feed and restore 1480 TPS / hood .300, retaining robot pose |
| Circle | Stop shooter/feed, restore those settings, and reset starting robot pose |

Holding a D-pad button does not repeat. Hood adjustments are servo-position
increments: increasing position decreases physical hood angle and increases
launch angle. Hood commands are bounded to physical 15–36 degrees using the
confirmed 355-degree / 90%-PWM / 30:148 geometry. PWM is explicitly 600–2400 us.
The motor target is clipped at zero and the controller output is limited to 0–1;
no unverified hardware maximum TPS is invented.

Settings are held fixed during a feed press. Turning the shooter on while holding
feed does not fire; release and press again. Both intake motors feed at **1100 TPS**,
using only the left intake encoder for velocity control.

## Collecting useful samples

1. Start near the 51.43-inch known shot, with hood .300 and target 1480 TPS.
2. Let the displayed turret angle converge to its target. Press Cross and wait
   for **Settled = true**: measured motor velocity must remain
   within 20 TPS of the target for at least 150 ms, with hood settings unchanged.
3. Press right bumper to feed one ball and release it promptly. A snapshot is
   captured **before the gate opens**: current pose, goal, distance, target TPS,
   measured pre-feed motor TPS, hood position/derived angle, and settled status.
4. Press Square if it scored, or Triangle if it missed. Repeat for five balls,
   waiting for recovery each time. Keep hood fixed while finding a useful TPS.
5. Repeat around 45, 55 and 70 inches with controlled hood settings. Record
   misses too. Do not infer a common entry angle across successful shots.
6. Press STOP normally to save the session CSV, then send that file for fitting.

This logs **feed attempts**, not automatically detected individual ball exits.
Holding feed long enough for multiple balls creates only one row. Press/release
for one ball per row and leave ambiguous attempts unrated. Feeding before settled
is allowed, but the row is tagged `settled=false`; those rows should not be used
as stabilized velocity-transfer calibration. The velocity measured when the gate
opens is a pre-feed sample, not a measured ball exit speed. No entry angle,
flight time, or maximum height is fabricated from these samples.

Only the latest unrated attempt can be rated. Starting another attempt retains
the earlier one as UNRATED. Resetting settings or pose preserves the collected
rows. Unrated and unsettled rows remain available for diagnosis.

## File output

The actual filename is shown on telemetry. Sessions use unique names such as:

```
/sdcard/FIRST/settings/shooter-calibration-1790000000000.csv
```

The device chooses the FIRST/settings location through the SDK's AppUtil. Copy
the displayed file from the Robot Controller, for example with USB/ADB:

```
adb pull /sdcard/FIRST/settings/shooter-calibration-<session>.csv
```

The CSV contains units in its headers, motor-encoder TPS without a gearing
conversion, the 104 cm goal height, PWM limits, and operator outcome. Serialization
and disk writing run after motors stop in cleanup; formatting and file I/O do not
run in the control loop. Until normal STOP, records are buffered in memory and
will be lost if the app/device is forcibly killed. Telemetry reports write failures.

Localization and velocity feedback run each loop, with one Pinpoint update and
manual hub bulk caching. Duplicate output commands are skipped, and telemetry
runs at 5 Hz. This does not certify hardware loop time without measurement.
