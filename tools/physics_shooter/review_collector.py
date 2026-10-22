"""Audit collected attempts and evaluate a conditional physics fit, without changing the LUT.

Usage: python tools/physics_shooter/review_collector.py path/to/collector.csv
Successful shots are approximate point-intersection observations, not measured
ball exit speeds. Distance holdout keeps repeated shots together.
"""
import argparse
import csv
import json
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np

import generate as physics


def metrics(errors):
    errors = np.asarray(errors)
    return {'rmse_tps': float(np.sqrt(np.mean(errors**2))),
            'mae_tps': float(np.mean(np.abs(errors))),
            'maximum_absolute_tps': float(np.max(np.abs(errors)))}


def location(distance_in):
    return round(distance_in/0.5)*0.5


def review(source):
    with source.open(newline='', encoding='utf-8-sig') as f:
        rows = list(csv.DictReader(f))
    invalid, accepted = [], []
    numeric = ['robot_x_in', 'robot_y_in', 'goal_x_in', 'goal_y_in', 'distance_in',
               'target_motor_tps', 'measured_pre_feed_motor_tps', 'hood_servo',
               'hood_degrees', 'target_height_cm', 'pwm_min_us', 'pwm_max_us']
    for row in rows:
        try:
            values = {key: float(row[key]) for key in numeric}
            if not all(np.isfinite(v) for v in values.values()):
                raise ValueError('nonfinite input')
            if row['result'] not in ('SCORE', 'MISS', 'UNRATED') or row['settled'] not in ('true', 'false'):
                raise ValueError('unknown outcome/settled flag')
            distance = np.hypot(values['goal_x_in']-values['robot_x_in'],
                                values['goal_y_in']-values['robot_y_in'])
            if abs(distance-values['distance_in']) > 1e-4:
                raise ValueError('logged distance does not match pose/goal')
            if (abs(values['target_height_cm']-physics.HEIGHT*100) > 1e-6 or
                    values['pwm_min_us'] != physics.PWM_MIN_US or values['pwm_max_us'] != physics.PWM_MAX_US):
                raise ValueError('height/PWM differs from this physics scenario')
            expected_hood = 15+(.485-values['hood_servo'])*physics.HOOD_DEGREES_PER_POSITION
            if abs(expected_hood-values['hood_degrees']) > 1e-4:
                raise ValueError('hood conversion differs from this physics scenario')
            accepted.append({**row, **values})
        except (KeyError, ValueError) as error:
            invalid.append({'attempt': row.get('attempt'), 'reason': str(error)})

    groups = defaultdict(list)
    for row in accepted:
        groups[(location(row['distance_in']), row['hood_servo'], row['target_motor_tps'])].append(row)
    settings = []
    for (distance, hood, target), group in sorted(groups.items()):
        settled = [r for r in group if r['settled'] == 'true']
        settled_scores = sum(r['result'] == 'SCORE' for r in settled)
        settled_misses = sum(r['result'] == 'MISS' for r in settled)
        settings.append(dict(location_bin_in=distance, distance_min_in=min(r['distance_in'] for r in group),
                             distance_max_in=max(r['distance_in'] for r in group), hood_servo=hood,
                             target_motor_tps=target, attempts=len(group),
                             outcomes=dict(Counter(r['result'] for r in group)),
                             settled_scores=settled_scores, settled_misses=settled_misses,
                             settled_rated=settled_scores+settled_misses,
                             mean_pre_feed_motor_tps=float(np.mean([r['measured_pre_feed_motor_tps'] for r in group]))))
    training = [r for r in accepted if r['settled'] == 'true' and r['result'] == 'SCORE']
    if len(training) < 3:
        raise ValueError('Need at least three valid settled scores to evaluate a fit')
    d = np.array([r['distance_in']*physics.INCH for r in training])
    angle = np.array([90-r['hood_degrees'] for r in training])
    tps = np.array([r['measured_pre_feed_motor_tps'] for r in training])
    speeds = physics.solve_speed(d, angle, .47, steps=240)
    trajectories = physics.simulate(d, angle, speeds, .47, steps=400)
    if not np.all(trajectories['valid'] & (trajectories['vz'] < 0)):
        raise ValueError('At least one scored shot is inconsistent with assumed descending point intersection')
    k, b = physics.fit_transfer(tps, speeds)
    current = json.loads((physics.ROOT/'doc/physics-shooter/calibration.json').read_text())['constants']
    current_errors = (speeds-current['b_mps'])/current['k_mps_per_tps']-tps
    errors = (speeds-b)/k-tps
    # Test a simpler physically constrained alternative rather than selecting by training error.
    proportional_k = float(np.dot(tps, speeds)/np.dot(tps, tps))
    proportional_errors = speeds/proportional_k-tps
    clusters = np.array([location(r['distance_in']) for r in training])
    held_out = []
    all_errors = []
    proportional_holdout = []
    for cluster in sorted(set(clusters)):
        fit = clusters != cluster
        hk, hb = physics.fit_transfer(tps[fit], speeds[fit])
        residual = (speeds[~fit]-hb)/hk-tps[~fit]
        pk = float(np.dot(tps[fit], speeds[fit])/np.dot(tps[fit], tps[fit]))
        proportional_holdout.extend((speeds[~fit]/pk-tps[~fit]).tolist())
        all_errors.extend(residual.tolist())
        held_out.append(dict(location_bin_in=float(cluster), samples=int(np.sum(~fit)),
                             fitted_k=hk, fitted_b=hb, **metrics(residual)))
    return dict(source=str(source.resolve()), status='REVIEW ONLY: active calibration and LUT unchanged',
                rows=len(rows), valid_rows=len(accepted), invalid_rows=invalid,
                outcomes=dict(Counter(r['result'] for r in rows)),
                unsettled_attempts=sum(r['settled'] == 'false' for r in rows),
                settled_scores_used=len(training), grouping_distance_step_in=.5, settings=settings,
                conditional_fit=dict(cd_assumed=.47, angle_offset_assumed_deg=0,
                                     k_mps_per_tps=k, b_mps=b, training=metrics(errors),
                                     original_model=metrics(current_errors), distance_holdout=metrics(all_errors),
                                     held_out_locations=held_out),
                zero_intercept_fit=dict(k_mps_per_tps=proportional_k, b_mps=0,
                                        training=metrics(proportional_errors),
                                        distance_holdout=metrics(proportional_holdout)),
                note='Scores are not precise measured trajectory intersections; misses lack directional error. Do not use repeated-shot training error as evidence of generalization to new distances or hood angles.')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('csv', type=Path)
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    report = review(args.csv)
    text = json.dumps(report, indent=2)+'\n'
    if args.report:
        args.report.write_text(text, encoding='utf-8')
    print(json.dumps({k: report[k] for k in ['rows', 'invalid_rows', 'outcomes', 'unsettled_attempts',
          'settled_scores_used', 'conditional_fit', 'zero_intercept_fit']}, indent=2))


if __name__ == '__main__':
    main()
