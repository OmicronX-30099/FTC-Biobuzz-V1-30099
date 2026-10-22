# Collected shooter calibration review

Source: `tools/physics_shooter/collected/shooter-calibration-1777690460337.csv`,
preserved byte-for-byte from the user's file on E:. No active shooting table or
physics calibration constants are changed by this review.

## Data quality

All 80 rows match the logged pose/goal distance, hood geometry, 104 cm target
height, and 600–2400 us PWM configuration. There are 47 SCORE, 22 MISS, and 11
UNRATED attempts. Five attempts are unsettled, including four SCORE attempts.
The conditional physics fit therefore uses **43 settled SCORE attempts**.
Misses and unrated attempts are retained in outcome summaries, not treated as
successful point intersections. No entry angle is imposed on historical shots.

The dataset covers four approximate distance groups: 45.0, 51.5, 54.5 and 70.5
inches. These 0.5-inch labels summarize nearby readings; actual distances are
used for fitting. Repeated shots are not independent new calibration distances.

## Repeated settings

These are operator-reported outcomes, not expected future success probabilities.
Counts below include only settled, rated attempts at the indicated target/hood.

| Actual distance (in, approximately) | Target motor TPS | Hood servo | Settled scores / rated attempts |
|---|---|---|---|
| 45.1 | 1420 | 0.360 | 7 / 8 |
| 51.4–51.5 | 1480 | 0.300 | 10 / 13 |
| 54.5 | 1460 | 0.300 | 7 / 7 |
| 70.4–70.5 | 1540 | 0.345 | 3 / 5 |

Other far settings are less tested or inconsistent. For example, around 70.4
inches, 1540 TPS / hood .270 gives 2 settled scores in 7 rated attempts; 1580 TPS /
hood .315 gives 2 in 3. The small sample counts do not justify declaring either
far setting reliable. At approximately the same range, shots were collected
from different sides of the goal, so aiming/localization differences may also
affect outcome; the file does not establish a cause for each miss.

Measured pre-feed velocity is used in the physical transfer fit, not the target
or a post-shot minimum. The log's velocities appear quantized in 20 TPS increments,
which limits their precision. Scores identify entry into the goal, not a precise
measurement of trajectory height at its center or of ball exit speed.

## Conditional physics refit and validation

Holding Cd at 0.47 and release-angle offset at zero, RK4/root-finding computes
required ball speeds separately at each row's own hood angle and actual distance.
A positive-slope affine fit to the 43 settled scores yields:

```
v_exit = 0.004559223306 * measuredMotorTPS - 1.321306529  [m/s]
```

This is a candidate local transfer model, not a new active calibration. Its
negative intercept must not be extrapolated into a stopped-motor regime.

| Comparison | TPS RMSE | TPS MAE |
|---|---|---|
| Current Desmos-derived model, evaluated on these 43 rows | 51.60 | 38.91 |
| Candidate fit, evaluated on its training rows | 24.98 | 20.49 |
| Candidate, leaving out a whole distance group before prediction | 196.92 | 116.67 |

Holding out the entire far-distance group is particularly poor: approximately
386 TPS RMSE. Holding out each nearer group gives about 26–32 TPS RMSE. Random
row holdout would hide this weakness by leaving nearly identical repeated shots
in the training data. The candidate is not robust enough to replace the LUT.

A simpler physically constrained alternative fixes b=0 and fits only k:
`v_exit = 0.003671396346 * motorTPS`. Training RMSE is 33.36 TPS; distance-holdout
RMSE is 42.13 TPS. This generalizes much better than the two-parameter affine
refit, despite its higher training error. It remains a candidate: the new steep
launch-angle region below is not represented by this collection, and scoring
scatter is still substantial at the far distance. It is stored in the analysis
JSON rather than silently adopted as a validated model.

Measured successful hood angles in this log are approximately 23.1–28.9 degrees.
The height-prioritized optimizer selects about 16–20 degrees at 45–55 inches.
The new collection still does not directly validate that steeper-launch region.
The rows also do not measure peak height, flight time, or entry angle; they cannot
verify the optimizer's 136 cm peak requirement or preferred 30-degree entry.

The result points toward a transfer/release model that needs validation across
hood settings, possible point-intersection approximation error, and ordinary
shot/aiming variation. It does not identify a new Cd or prove a particular cause.
Adding more free parameters solely to reduce training error is not justified.

## Practical next comparison

Use the repeated settings above as measured baselines. For the physics-generated
shot near 51.4 inches, select hood **.430** in the collector (the nearest .005
control step to the physics command .4314), keep it fixed, and find a
repeatable scoring TPS with the collector. Record five rated attempts at one
setting, then repeat the same hood at another distance. This directly tests the
currently unvalidated high-launch-angle transfer instead of only gathering more
repeats at .300 hood. Video of one launch and trajectory, or a measured ball exit
speed, would help separate release-angle error from speed-transfer and drag error.

Complete setting counts and conditional-fit diagnostics: `collector-review.json`.
Reproduce the analysis without modifying the LUT:

```
python tools/physics_shooter/review_collector.py tools/physics_shooter/collected/shooter-calibration-1777690460337.csv --report doc/physics-shooter/collector-review.json
```
