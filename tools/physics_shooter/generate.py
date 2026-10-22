"""Offline quadratic-drag calibration/optimization. Never imported by the robot.

Run from the repository root: python tools/physics_shooter/generate.py
Unknown historical hood angles remain blank in measurements.csv. Their angles
are derived from the user's linear servo/PWM/gear geometry, not marked measured.
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
INCH = 0.0254
G = 9.80665
HEIGHT = 1.04
MIN_VERTEX = 1.35
SAFETY_MARGIN = 0.01
PREFERRED_ENTRY_DEG = 30.0
HOOD_MIN, HOOD_MAX = 15.0, 36.0
RHO = 1.225  # assumed sea-level air density, kg/m^3
DIAMETER = 2.8 * INCH  # AndyMark BIOBUZZ Pollen nominal diameter
MASS = 0.055 * 0.45359237  # AndyMark nominal 0.055 lb, approximately 25 g
SERVO_15 = 0.485
SERVO_FULL_TRAVEL_DEG = 355.0
PWM_MIN_US, PWM_MAX_US = 600.0, 2400.0
SERVO_FULL_PWM_MIN_US, SERVO_FULL_PWM_MAX_US = 500.0, 2500.0
HOOD_DEGREES_PER_POSITION = (SERVO_FULL_TRAVEL_DEG *
    (PWM_MAX_US-PWM_MIN_US)/(SERVO_FULL_PWM_MAX_US-SERVO_FULL_PWM_MIN_US) * 30.0/148.0)
ANGLE_OFFSET = 0.0
AREA = np.pi * DIAMETER**2 / 4


def ideal_speed(distance, angle_deg):
    """Analytic speed required to intersect (d, HEIGHT); meters and radians inside."""
    d, a = np.asarray(distance), np.deg2rad(angle_deg)
    denominator = 2 * np.cos(a)**2 * (d * np.tan(a) - HEIGHT)
    return np.sqrt(np.divide(G * d**2, denominator,
                            out=np.full(np.broadcast_shapes(d.shape, a.shape), np.nan),
                            where=denominator > 0))


def simulate(distance, angle_deg, speed, cd=0.47, steps=160, trace=False):
    """RK4 with x as independent variable; vx stays positive for valid shots.

    State = (z, vx, vz, elapsed time). dx can vary per vectorized trajectory.
    This ends exactly at the target's x, avoiding a time-step target overshoot.
    Vertex is interpolated at vz=0 between consecutive RK4 samples.
    """
    d, a, speed = np.broadcast_arrays(np.asarray(distance, dtype=float),
                                      np.deg2rad(angle_deg), np.asarray(speed, dtype=float))
    state = np.stack((np.zeros_like(d), speed*np.cos(a), speed*np.sin(a), np.zeros_like(d)))
    q = 0.5 * RHO * cd * AREA / MASS
    dx = d / steps
    vertex = np.zeros_like(d)
    vertex_x = np.zeros_like(d)
    valid = np.isfinite(speed) & (speed > 0) & (d > 0)
    history = [(np.zeros_like(d), state[0].copy())] if trace else None

    def derivative(s):
        vx = np.maximum(s[1], 1e-6)
        magnitude = np.hypot(s[1], s[2])
        return np.stack((s[2]/vx, -q*magnitude,
                         (-G-q*magnitude*s[2])/vx, 1/vx))

    with np.errstate(over='ignore', invalid='ignore', divide='ignore'):
        for i in range(steps):
            old = state
            k1 = derivative(old)
            k2 = derivative(old + dx*k1/2)
            k3 = derivative(old + dx*k2/2)
            k4 = derivative(old + dx*k3)
            state = old + dx*(k1+2*k2+2*k3+k4)/6
            valid &= (state[1] > 1e-5) & np.all(np.isfinite(state), axis=0)
            crossing = (old[2] > 0) & (state[2] <= 0) & valid
            f = np.clip(np.divide(old[2], old[2]-state[2],
                                 out=np.zeros_like(d), where=(old[2] != state[2])), 0, 1)
            # Interpolate vz=0; integrate local slope for the apex height.
            apex_z = old[0] + 0.5*(old[2]/np.maximum(old[1], 1e-6))*dx*f
            vertex = np.where(crossing, apex_z, np.maximum(vertex, state[0]))
            vertex_x = np.where(crossing, (i+f)*dx, vertex_x)
            # Keep failed low-speed trial trajectories finite for bisection.
            state = np.where(valid[None, ...], state, 0.0)
            if trace:
                history.append(((i+1)*dx, state[0].copy()))
    result = dict(z=np.where(valid, state[0], -1e6), vx=state[1], vz=state[2],
                  time=state[3], vertex=vertex, vertex_x=vertex_x, valid=valid)
    result['entry'] = np.rad2deg(np.arctan2(-state[2], state[1]))
    if trace:
        result['trajectory'] = np.asarray(history, dtype=float)
    return result


def solve_speed(distance, angle_deg, cd, steps=100):
    """Bracket then bisect exit speed to hit the target. No arbitrary motor cap."""
    d, angle = np.broadcast_arrays(np.asarray(distance, dtype=float), np.asarray(angle_deg, dtype=float))
    estimate = ideal_speed(d, angle)
    low = np.zeros_like(d)
    high = np.maximum(estimate*1.5, 8)
    bracketed = np.isfinite(estimate)
    for _ in range(10):
        sim = simulate(d, angle, high, cd, steps)
        needs_more = sim['z'] < HEIGHT
        if not np.any(needs_more):
            break
        high = np.where(needs_more, high*2, high)
    sim = simulate(d, angle, high, cd, steps)
    bracketed &= sim['valid'] & (sim['z'] >= HEIGHT)
    for _ in range(23):
        mid = (low+high)/2
        z = simulate(d, angle, mid, cd, steps)['z']
        low = np.where(z < HEIGHT, mid, low)
        high = np.where(z >= HEIGHT, mid, high)
    return np.where(bracketed, (low+high)/2, np.nan)


def read_measurements():
    with (Path(__file__).parent/'measurements.csv').open(newline='') as f:
        rows = list(csv.DictReader(f))
    for row in rows:
        for key in ('robot_x_in', 'robot_y_in', 'tps', 'hood_servo'):
            row[key] = float(row[key])
        row['distance_in'] = float(np.hypot(row['robot_x_in']-56, row['robot_y_in']-59))
        row['include_in_fit'] = row['include_in_fit'] == 'true'
        measured = row['measured_hood_deg']
        row['hood_source'] = 'measured' if measured else 'derived_from_user_geometry'
        row['hood_deg'] = float(measured) if measured else (
            HOOD_MIN + (SERVO_15-row['hood_servo'])*HOOD_DEGREES_PER_POSITION)
    return rows


def fit_transfer(tps, speeds):
    """Positive slope bounded least squares, intercept unrestricted within positive tested speeds.

    This affine transfer is a local calibration, never extrapolated to stopped motors.
    """
    tps, speeds = np.asarray(tps), np.asarray(speeds)
    k, b = np.linalg.lstsq(np.column_stack((tps, np.ones_like(tps))), speeds, rcond=None)[0]
    if not np.isfinite(k+b) or k <= 0 or np.any(k*tps+b <= 0):
        raise ValueError('Data do not support a positive affine velocity transfer')
    return float(k), float(b)


def optimize(distances_in, cd, k, b, hood_step=0.1):
    hoods = np.linspace(HOOD_MIN, HOOD_MAX, round((HOOD_MAX-HOOD_MIN)/hood_step)+1)
    d = np.asarray(distances_in)[:, None]*INCH
    angles = 90-hoods[None, :]+ANGLE_OFFSET
    print(f'Optimizing {d.size} distances x {hoods.size} hood settings, Cd={cd}', flush=True)
    speeds = ideal_speed(d, angles) if cd == 0 else solve_speed(d, angles, cd)
    sim = simulate(d, angles, speeds, cd, steps=160)
    tps = (speeds-b)/k
    valid = (sim['valid'] & (sim['vz'] < 0) & (sim['vertex_x'] > 0)
             & (sim['vertex_x'] < d) & (np.abs(sim['z']-HEIGHT) < 0.0001)
             & np.isfinite(tps) & (tps > 0))
    # A small numerical reserve keeps grid/interpolation roundoff away from 136 cm.
    height_ok = valid & (sim['vertex'] >= MIN_VERTEX+SAFETY_MARGIN+0.0001)
    output = []
    for i, distance in enumerate(distances_in):
        candidates = np.flatnonzero(height_ok[i])
        feasible = bool(candidates.size)
        if feasible:
            errors = np.abs(sim['entry'][i, candidates]-PREFERRED_ENTRY_DEG)
            # Entry errors within 0.01 degree are indistinguishable for this model;
            # lower motor speed breaks the tie only AFTER the hard constraints.
            candidates = candidates[errors <= errors.min()+0.01]
            chosen = candidates[np.argmin(tps[i, candidates])]
        else:
            candidates = np.flatnonzero(valid[i])
            if not candidates.size:
                raise ValueError(f'No descending solution at {distance} in')
            chosen = candidates[np.argmax(sim['vertex'][i, candidates])]
        output.append(dict(distance_in=float(distance), hood_deg=float(hoods[chosen]),
                           launch_deg=float(angles[0, chosen]), tps=float(tps[i, chosen]),
                           exit_mps=float(speeds[i, chosen]), entry_deg=float(sim['entry'][i, chosen]),
                           vertex_cm=float(sim['vertex'][i, chosen]*100),
                           vertex_in=float(sim['vertex_x'][i, chosen]/INCH),
                           flight_s=float(sim['time'][i, chosen]), feasible=feasible,
                           height_constraint_satisfied=feasible,
                           entry_error_deg=float(abs(sim['entry'][i, chosen]-PREFERRED_ENTRY_DEG)),
                           target_error_mm=float((sim['z'][i, chosen]-HEIGHT)*1000)))
    return output


def validate_table(rows, cd, k, b):
    """Independently integrate interpolated midpoint commands at finer resolution."""
    d = (np.array([r['distance_in'] for r in rows[:-1]])+
         np.array([r['distance_in'] for r in rows[1:]]))/2
    hood = (np.array([r['hood_deg'] for r in rows[:-1]])+
            np.array([r['hood_deg'] for r in rows[1:]]))/2
    tps = (np.array([r['tps'] for r in rows[:-1]])+
           np.array([r['tps'] for r in rows[1:]]))/2
    sim = simulate(d*INCH, 90-hood+ANGLE_OFFSET, k*tps+b, cd, steps=400)
    feasible = np.array([a['feasible'] and b['feasible'] for a, b in zip(rows, rows[1:])])
    error = float(np.max(np.abs(sim['z']-HEIGHT))*1000)
    if not np.all(sim['valid'] & (sim['vz'] < 0)) or error > 1.0:
        raise AssertionError(f'LUT midpoint miss exceeds 1 mm: {error}')
    if np.any(sim['vertex'][feasible] < MIN_VERTEX+SAFETY_MARGIN):
        raise AssertionError('Interpolated shot falsely claimed height feasibility')
    return dict(midpoint_max_target_error_mm=error,
                minimum_feasible_midpoint_vertex_cm=float(np.min(sim['vertex'][feasible])*100))


def export_kotlin(rows, k, b, cd):
    folder = ROOT/'TeamCode/src/main/java/org/firstinspires/ftc/teamcode/experimental/physics'
    folder.mkdir(parents=True, exist_ok=True)
    text = ('package org.firstinspires.ftc.teamcode.experimental.physics\n\n'
            '// Generated by tools/physics_shooter/generate.py. Do not edit this table.\n'
            '// PROVISIONAL aerodynamic calibration; hood mapping uses user-supplied servo geometry.\n'
            'internal object PhysicsShotTable {\n'
            f'    const val K = {k:.12g}\n    const val B = {b:.12g}\n'
            f'    const val CD = {cd}\n    const val ANGLE_OFFSET_DEG = {ANGLE_OFFSET}\n'
            f'    const val TARGET_HEIGHT_CM = {HEIGHT*100}\n'
            f'    const val PREFERRED_ENTRY_DEG = {PREFERRED_ENTRY_DEG}\n'
            f'    const val HOOD_DEGREES_PER_POSITION = {HOOD_DEGREES_PER_POSITION:.15g}\n'
            f'    const val PWM_MIN_US = {PWM_MIN_US}\n    const val PWM_MAX_US = {PWM_MAX_US}\n'
            f'    const val STEP_IN = {rows[1]["distance_in"]-rows[0]["distance_in"]}\n')
    for name, key in [('distance', 'distance_in'), ('hood', 'hood_deg'), ('tps', 'tps'),
                      ('entry', 'entry_deg'), ('vertexCm', 'vertex_cm'), ('vertexIn', 'vertex_in'),
                      ('flight', 'flight_s'), ('speed', 'exit_mps')]:
        text += f'    val {name} = doubleArrayOf(\n'
        for start in range(0, len(rows), 6):
            text += '        '+', '.join(f'{r[key]:.10f}' for r in rows[start:start+6])+',\n'
        text += '    )\n'
    text += '    val feasible = booleanArrayOf(\n'
    for start in range(0, len(rows), 16):
        text += '        '+', '.join(str(r['feasible']).lower() for r in rows[start:start+16])+',\n'
    text += '    )\n}\n'
    (folder/'PhysicsShotTable.kt').write_text(text, encoding='utf-8')


def make_plots(out, ideal, drag, measurements, k, b, cd):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.size': 10, 'figure.dpi': 160})
    fig, axes = plt.subplots(2, 2, figsize=(11, 8), constrained_layout=True)
    for ax, field, label in zip(axes.flat, ['hood_deg', 'tps', 'entry_deg', 'vertex_cm'],
                               ['Hood angle (deg)', 'Shooter velocity (TPS)',
                                'Downward entry angle (deg)', 'Vertex above exit (cm)']):
        for model, title, color in [(ideal, 'Ideal, independently calibrated', '#999999'),
                                    (drag, 'Quadratic drag, provisional calibration', '#2068ad')]:
            ax.plot([r['distance_in'] for r in model], [r[field] for r in model],
                    label=title, color=color, linestyle='--' if model is ideal else '-')
        infeasible = [r for r in drag if not r['feasible']]
        ax.scatter([r['distance_in'] for r in infeasible], [r[field] for r in infeasible],
                   s=6, color='#c05242', label='Drag: height infeasible')
        if field == 'entry_deg':
            ax.axhline(PREFERRED_ENTRY_DEG, color='black', linestyle=':', linewidth=1,
                       label=f'Preferred {PREFERRED_ENTRY_DEG:g} degrees down')
            ax.legend(fontsize=7)
        if field == 'vertex_cm':
            ax.axhline(135, color='black', linestyle=':', label='135 cm minimum')
            ax.axhline(136, color='green', linestyle=':', label='136 cm with margin')
        ax.set(xlabel='Horizontal distance (in)', ylabel=label)
        ax.grid(alpha=.2)
    axes[0, 0].legend(fontsize=7)
    axes[1, 1].legend(fontsize=7)
    fig.suptitle(f'Experimental physics shots — user servo geometry, assumed Cd={cd}')
    fig.savefig(out/'optimization.png')
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(9, 7), constrained_layout=True)
    for row in measurements:
        color = '#2068ad' if row['include_in_fit'] else '#c05242'
        axes[0].scatter(row['distance_in'], row['tps'], color=color, marker='o')
        axes[0].scatter(row['distance_in'], row['predicted_tps'], color=color, marker='x')
        axes[1].bar(row['distance_in'], row['residual_tps'], width=1.2, color=color)
    axes[0].set(ylabel='Motor TPS', title='Observed hood settings: circles measured, crosses predicted\nRed row excluded by user; physical angles derived from servo geometry')
    axes[1].axhline(0, color='black', linewidth=.8)
    axes[1].set(xlabel='Horizontal distance (in)', ylabel='Predicted − measured TPS')
    fig.savefig(out/'calibration.png')
    plt.close(fig)

    fig, axes = plt.subplots(1, 2, figsize=(12, 5), constrained_layout=True)
    for ax, model, model_cd, title in [(axes[0], ideal, 0, 'Ideal projectile'),
                                       (axes[1], drag, cd, 'Quadratic drag (provisional)')]:
        for distance in (35, 45, 55, 65, 75):
            row = min(model, key=lambda r: abs(r['distance_in']-distance))
            sim = simulate(distance*INCH, row['launch_deg'], row['exit_mps'], model_cd, 400, True)
            xy = sim['trajectory']
            np.savetxt(out/f'trajectory_{"ideal" if model_cd == 0 else "drag"}_{distance}in.csv',
                       np.column_stack((xy[:, 0], xy[:, 1])), delimiter=',',
                       header='horizontal_m,height_m', comments='')
            line, = ax.plot(xy[:, 0]/INCH, xy[:, 1]*100, label=f'{distance} in'+(' *' if not row['feasible'] else ''))
            ax.scatter(distance, HEIGHT*100, color=line.get_color(), marker='s', s=20)
            ax.scatter(row['vertex_in'], row['vertex_cm'], color=line.get_color(), marker='x')
        ax.axhline(135, color='black', linestyle=':', label='135 cm')
        ax.set(title=title, xlabel='Horizontal distance (in)', ylabel='Height above shooter (cm)')
        ax.legend(fontsize=7, title='* height infeasible')
        ax.grid(alpha=.2)
    fig.savefig(out/'trajectories.png')
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--cd', type=float, default=0.47)
    parser.add_argument('--distance-step', type=float, default=0.25)
    parser.add_argument('--hood-step', type=float, default=0.1)
    args = parser.parse_args()
    if not (0 <= args.cd <= 2 and 0 < args.distance_step <= 1 and 0 < args.hood_step <= .25):
        parser.error('Use Cd 0..2, distance step (0,1], hood step (0,.25]')
    if abs(40/args.distance_step-round(40/args.distance_step)) > 1e-8:
        parser.error('Distance step must divide the 35–75 inch range exactly')
    out = ROOT/'doc/physics-shooter'
    out.mkdir(parents=True, exist_ok=True)
    measurements = read_measurements()
    d = np.array([r['distance_in']*INCH for r in measurements])
    angle = np.array([90-r['hood_deg']+ANGLE_OFFSET for r in measurements])
    measured_tps = np.array([r['tps'] for r in measurements])
    included = np.array([r['include_in_fit'] for r in measurements])
    speeds = solve_speed(d, angle, args.cd, steps=240)
    observed = simulate(d, angle, speeds, args.cd, steps=400)
    if not np.all(observed['valid'] & (observed['vz'] < 0)):
        raise ValueError('At least one historical shot is incompatible with descending entry')
    k, b = fit_transfer(measured_tps[included], speeds[included])
    ideal_k, ideal_b = fit_transfer(measured_tps[included], ideal_speed(d, angle)[included])
    predicted = (speeds-b)/k
    for i, row in enumerate(measurements):
        row.update(required_exit_mps=float(speeds[i]), predicted_tps=float(predicted[i]),
                   residual_tps=float(predicted[i]-row['tps']),
                   absolute_error_tps=float(abs(predicted[i]-row['tps'])),
                   absolute_error_percent=float(abs(predicted[i]-row['tps'])/row['tps']*100))
    # Leave-one-out validation re-fits transfer only; Cd and hood mapping stay fixed assumptions.
    loo, interpolation_errors = [], []
    for i in np.flatnonzero(included):
        training = included.copy()
        training[i] = False
        lk, lb = fit_transfer(measured_tps[training], speeds[training])
        loo.append(float((speeds[i]-lb)/lk-measured_tps[i]))
        order = np.argsort(d[training])
        baseline = np.interp(d[i], d[training][order], measured_tps[training][order])
        interpolation_errors.append(float(baseline-measured_tps[i]))
    distances = np.linspace(35, 75, round(40/args.distance_step)+1)
    ideal = optimize(distances, 0, ideal_k, ideal_b, args.hood_step)
    drag = optimize(distances, args.cd, k, b, args.hood_step)
    validation = validate_table(drag, args.cd, k, b)
    def metrics(errors):
        return dict(rmse_tps=float(np.sqrt(np.mean(np.square(errors)))),
                    mae_tps=float(np.mean(np.abs(errors))))
    report = dict(status='PROVISIONAL: effective drag and actual launch-angle offset remain unmeasured',
                  user_confirmation=dict(game='BIOBUZZ', ball_diameter_in=2.8, ball_mass_g=25,
                                         servo_full_travel_deg=SERVO_FULL_TRAVEL_DEG,
                                         pwm_min_us=PWM_MIN_US, pwm_max_us=PWM_MAX_US,
                                         servo_full_pwm_min_us=SERVO_FULL_PWM_MIN_US,
                                         servo_full_pwm_max_us=SERVO_FULL_PWM_MAX_US,
                                         hood_gear_ratio='30/148',
                                         hood_degrees_per_position=HOOD_DEGREES_PER_POSITION,
                                         servo_0_16_hood_deg=15+(.485-.16)*HOOD_DEGREES_PER_POSITION,
                                         linkage_linearity_user_confirmed=True),
                  constants=dict(k_mps_per_tps=k, b_mps=b, cd_assumed=args.cd,
                                 angle_offset_assumed_deg=ANGLE_OFFSET, rho_kg_m3=RHO,
                                 diameter_m=DIAMETER, mass_kg=MASS, target_height_m=HEIGHT,
                                 minimum_vertex_m=MIN_VERTEX, safety_margin_m=SAFETY_MARGIN,
                                 preferred_entry_angle_deg=PREFERRED_ENTRY_DEG),
                  training=metrics((predicted-measured_tps)[included]),
                  leave_one_out=metrics(loo),
                  interpolation_leave_one_out=metrics(interpolation_errors),
                  interpolation_training=dict(rmse_tps=0, mae_tps=0),
                  validation=validation, measurements=measurements,
                  ideal_transfer=dict(k=ideal_k, b=ideal_b),
                  first_feasible_in=next((r['distance_in'] for r in drag if r['feasible']), None))
    for name, data in [('ideal.csv', ideal), ('drag.csv', drag), ('measurement_comparison.csv', measurements)]:
        with (out/name).open('w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=list(data[0]))
            writer.writeheader()
            writer.writerows(data)
    (out/'calibration.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    export_kotlin(drag, k, b, args.cd)
    make_plots(out, ideal, drag, measurements, k, b, args.cd)
    lines = ['| Distance (in) | Hood (deg) | TPS | Entry (deg down) | Vertex (cm) | Height feasible |',
             '|---|---|---|---|---|---|']
    for distance in range(35, 76, 5):
        r = min(drag, key=lambda r: abs(r['distance_in']-distance))
        lines.append(f'| {distance} | {r["hood_deg"]:.1f} | {r["tps"]:.1f} | {r["entry_deg"]:.2f} | {r["vertex_cm"]:.2f} | {r["feasible"]} |')
    (out/'predictions.md').write_text('\n'.join(lines)+'\n', encoding='utf-8')
    print(json.dumps({key: report[key] for key in ('constants', 'training', 'leave_one_out',
          'interpolation_leave_one_out', 'validation', 'first_feasible_in')}, indent=2), flush=True)


if __name__ == '__main__':
    main()
