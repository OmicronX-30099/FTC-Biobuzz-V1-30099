# First field trajectory observation

Reported robot XY: (59.35,7.68) inches. Configured goal 1 XY: (56,59) inches.
The horizontal goal distance is **51.4292 inches**.

The current unadjusted LUT at this distance predicts:

| Quantity | Nominal prediction |
|---|---|
| Motor target | 1570.41 TPS |
| Hood / servo command | 18.4717 degrees / 0.431394 |
| Launch angle | 71.5283 degrees above horizontal |
| Entry at target height | 57.0125 degrees downward |
| Vertex above shooter exit | 136.0639 cm / 53.5685 inches |
| Flight time to target height at goal XY | 0.77688 seconds |

These are interpolated model diagnostics. The user confirms multiplier 1 and
offset 0; the resulting nominal command is approximately 1570 TPS / servo 0.4314.
Settled motor velocity and actual hood position have not been independently logged.

The user observes about 75 inches peak height, Y=71 landing, about 60 degrees
downward entry by eye, and approximately 0.87–0.97 seconds flight with variation.
The user clarifies that the 75-inch peak is **above the shooter exit** and that
Y=71 means crossing the goal's **104-cm-above-exit height**, not floor impact.
Shooter exit is about 10 inches above the floor; no floor-height conversion is
applied to these reported heights. Flight time is taken to end at the reported
goal-height crossing.

This is a true 12-inch overshoot in Y at the goal's height. Assuming the same aim bearing,
the corresponding horizontal range is 63.4548 inches, approximately 23.4% longer
than the requested range. Landing X has not been reported.

The estimated 60-degree entry is already close to the nominal 57-degree entry;
the 30-degree preference is subordinate to the 136 cm peak requirement. It should
not be interpreted as a guaranteed 30-degree trajectory at this distance.

Preserved raw observation: `tools/physics_shooter/field_observations.json`.
It is excluded from fitting pending more reliable simultaneous measurements and
settled motor/hood telemetry. No calibration constants or LUT commands have been
changed from this observation. The historical Desmos shots had varying, unmeasured
entry angles; calibration does not impose a common entry angle on them.

## Conditional consistency check

`tools/physics_shooter/analyze_observation.py` runs the existing RK4 model without
changing its parameters. With Cd held at 0.47 and the same aim bearing:

- Keeping nominal launch angle 71.53 degrees and increasing exit speed from
  5.639 to 6.119 m/s explains the observed range. It predicts a 62.36-inch peak,
  62.07-degree entry, and 0.895-second flight. This agrees reasonably with the
  estimated entry/time, but not the reported 75-inch peak.
- Matching both the 75-inch peak and range instead requires about 75.17-degree
  actual launch and 6.629 m/s exit speed. It predicts 70.14-degree entry and a
  1.037-second flight, outside the reported angle/time estimates.

Thus a single speed multiplier cannot reconcile all reported values under the
current assumed drag model. This does not establish which observation or model
assumption is wrong: peak, angle, and time are approximate and may describe
different shots. The inferred angles are diagnostic actual release angles, not
commands that expand the mechanical hood limits. Exact conditional outputs are
in `observation-diagnosis.json`.
