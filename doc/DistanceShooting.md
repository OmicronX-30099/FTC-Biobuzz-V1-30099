# Distance-based shooter calibration

TeleOp now commands a paired flywheel TPS and hood servo position from the
distance to the selected goal. Measured robot Y above 70.75 inches selects goal 2;
otherwise goal 1 is selected. Cross no longer selects goals in TeleOp. Goal
coordinates are configured in `Util/Alliance.kt` and used for both turret aiming
and shot distance.

TeleOp supplies a predicted shooting position `(x + vx * 0.5, y + vy * 0.5)`
using field velocity in inches/second. Turret heading is predicted separately as
`heading + angularVelocity * 0.05`, using counterclockwise-positive radians/second
from Pedro's Pinpoint velocity. This 50 ms lead anticipates turret response during
rotation, following Meta Infinity's approach. Edit `ANGULAR_LEAD_SECONDS` in
`control/ShootingPosePrediction.java` to tune it; Dashboard access remains disabled.
Both turret
aiming and this distance table use that predicted position. The localizer pose
stays measured, and goal selection uses measured Y. This is the requested fixed
0.5-second lead, not a projectile time-of-flight calculation. At zero velocity,
the calibration distances and settings below apply directly.

D-pad left/right correct the localized robot heading by -2/+2 degrees per press,
preserving X/Y and the previous left/right aim direction. This correction is
visible in robot heading telemetry and is applied once to localization, rather
than added to the turret target. D-pad up/down add/subtract 20 ticks/second per
press from the selected flywheel target (clipped at zero). Hood remains based on
distance. Holding a button does not repeat. Circle restores the starting pose
and clears both accumulated offsets, including when a D-pad button is held.

## Calibration and model choice

Four supplied screenshot rows are currently used; the 56.45-inch row has been
removed from the active calibration. The Desmos graph
uses `(57, 59)` in its distance equation. The confirmed robot goal remains
`(56, 59)`, so distances below are recomputed from the original robot X/Y values
using `d = sqrt((x - 56)^2 + (y - 59)^2)`.

| Robot X (in) | Robot Y (in) | Distance (in) | Shooter TPS | Hood servo position |
| --- | --- | --- | --- | --- |
| 79.43 | 31.51 | 36.120146733 | 1440 | 0.485 |
| 55.76 | 7.56 | 51.440559873 | 1480 | 0.300 |
| 31.90 | 7.47 | 56.887176938 | 1540 | 0.250 |
| 8.20 | 7.30 | 70.411149685 | 1600 | 0.250 |

Use piecewise linear interpolation for both outputs. Between two adjacent
measurements `i` and `i+1`:

```
t = (d - D[i]) / (D[i+1] - D[i])
TPS(d)  = V[i] + t * (V[i+1] - V[i])
hood(d) = H[i] + t * (H[i+1] - H[i])
```

Here `D`, `V`, and `H` are the distance, TPS and servo-position columns in the
sorted table. The same equations can be entered as three distance intervals in
Desmos, with constant endpoint settings outside the calibrated range.

For example, at the midpoint between the two closest measurements
(`43.780353303` inches), the commands are `1460 TPS` and hood `0.3925`.
The close shot remains exactly `1440 TPS / 0.485` at its measured distance.

This treats the close-range high lob as an intentional calibration point, not
an outlier. A single global linear regression would not preserve all four
active measured combinations. Shots with hood and distance changing together do
not identify a reliable general TPS(distance, hood) surface; here the model
selects both commands together from distance. Hood values are logical servo
positions, not a linear conversion to hood or launch angle.

## Operating and adjusting it

- Dashboard configuration access is disabled. Edit settings in source and rebuild/reinstall.
- `Flywheel.AUTO_DISTANCE_SHOT` defaults to `true` in source.
- In auto mode, `ShotCalibration` stores `DISTANCE_INCHES`, `VELOCITY_TPS`, and
  `HOOD_POSITIONS` arrays. The same index in each array represents one shot.
  Keep lengths equal, at least two rows, and distances strictly increasing.
- Below `36.12` inches, hold `1440 TPS / 0.485`. Above `70.41` inches, hold
  `1600 TPS / 0.25`. Telemetry identifies when an endpoint is held. Those
  outside-range shots have not been calibrated; no ballistics extrapolation is
  implied.
- Set `Flywheel.AUTO_DISTANCE_SHOT = false` in source for manual control through
  `TARGET_VELOCITY_TPS` and `HOOD_POSITION`. In auto mode, those two manual fields
  do not override the calculated shot.
- All hood commands are clipped to the calibrated servo range `0.16–0.485`
  (`HOOD_MAX` through `HOOD_15_DEGREES`), including manual commands and INIT.
- TeleOp telemetry shows distance, mode/range status, target and measured TPS,
  and commanded hood position. Invalid pose/table input stops the flywheel
  and retains the last valid hood command.
- Edit source defaults in `ShotCalibration` or `Flywheel` to change settings.

## Measurements to collect next

With the 56.45-inch row removed, hood now changes from `0.30` to `0.25` over
about `5.45` inches, between `51.44` and `56.89` inches. The close-lob transition also
spans about 15.3 inches with no intermediate measurements. Test intermediate
distances, especially these two regions, and add or revise paired samples.

The table reproduces measured commands and generates intermediate candidates;
it does not establish that every interpolated or endpoint-clamped shot scores.
Correct the previously observed rotation-induced odometry error before relying
on automatic distance selection. The same calibration is used for both selected
goals, assuming equivalent goal geometry.
