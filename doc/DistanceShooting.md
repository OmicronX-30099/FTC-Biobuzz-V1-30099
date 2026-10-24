# Distance-based shooter calibration

TeleOp commands flywheel TPS and hood servo position by linearly interpolating
the paired calibration against horizontal distance from the robot to the
selected goal. Measured robot Y above 70.75 inches selects goal 2; otherwise goal
1 is selected. Goal coordinates are configured in `Util/Alliance.kt` and used
for turret aiming and shot distance.

D-pad left/right add -2/+2 degrees per press to the turret's aim target without
changing localized robot heading. D-pad up/down add/subtract 20 ticks/second per
press from flywheel TPS (clipped at zero). Holding a button does not repeat.
Circle restores the starting pose and clears both accumulated offsets.

## Calibration and model choice

The current calibration from the supplied table is:

| Distance from goal (in) | Shooter TPS | Hood servo position |
| --- | ---: | ---: |
| 23 | 1380 | 0.485 |
| 28 | 1420 | 0.450 |
| 38 | 1410 | 0.300 |
| 48 | 1480 | 0.250 |
| 58 | 1560 | 0.200 |
| 68 | 1580 | 0.200 |

Use piecewise linear interpolation for both outputs. Between two adjacent
measurements `i` and `i+1`:

```
t = (d - D[i]) / (D[i+1] - D[i])
TPS(d)  = V[i] + t * (V[i+1] - V[i])
hood(d) = H[i] + t * (H[i+1] - H[i])
```

Here `D`, `V`, and `H` are the distance, TPS and servo-position columns in the
sorted table. The same equations can be entered as five distance intervals in
Desmos, with constant endpoint settings outside the calibrated range.

Outside the calibrated 23–68 inch range, the nearest endpoint settings are
held. Hood values are logical servo positions. Both outputs are selected from
the same distance interval so each measured TPS/hood pair remains together.

## Operating and adjusting it

- `Flywheel.AUTO_DISTANCE_SHOT` defaults to `true`. Set it to `false` for manual
  control through `TARGET_VELOCITY_TPS` and `HOOD_POSITION`.
- In auto mode, `ShotCalibration` stores `DISTANCE_INCHES`, `VELOCITY_TPS`, and
  `HOOD_POSITIONS` arrays. The same index in each array represents one shot.
  Keep lengths equal, at least two rows, and distances strictly increasing.
- Below `23` inches, hold `1380 TPS / 0.485`. Above `68` inches, hold
  `1580 TPS / 0.20`. Telemetry identifies when an endpoint is held. Those
  outside-range shots have not been calibrated.
- In auto mode, `TARGET_VELOCITY_TPS` and `HOOD_POSITION`
  do not override the calculated shot.
- All hood commands are clipped to the calibrated servo range `0.16–0.485`
  (`HOOD_MAX` through `HOOD_15_DEGREES`), including manual commands and INIT.
- TeleOp telemetry shows distance to the selected goal, mode/range status, target and measured TPS,
  and commanded hood position. Invalid pose/table input stops the flywheel
  and retains the last valid hood command.
- Edit source defaults in `ShotCalibration` or `Flywheel` to change settings.

## Calibration notes

The table reproduces measured commands and generates intermediate candidates;
it does not establish that every interpolated or endpoint-held shot scores. The
same calibration is used for both selected goals, assuming equivalent goal
geometry.
