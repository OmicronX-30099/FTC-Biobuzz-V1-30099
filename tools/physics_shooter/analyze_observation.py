"""Diagnose a field observation without modifying the production or experimental LUT.

Range uses the current goal bearing because landing X was not measured. Cd stays
fixed; fitted launch speed/angle here are conditional explanations, not calibration.
"""
import csv
import json
from pathlib import Path

import numpy as np

import generate as physics


def main():
    source = Path(__file__).parent/'field_observations.json'
    observed = json.loads(source.read_text())['observations'][0]
    x = observed['robot_position_inches']['x']
    y = observed['robot_position_inches']['y']
    target_y = observed['reported_target_y_inches']
    requested_range = float(np.hypot(x-56, y-target_y))
    actual_range = requested_range * (observed['reported_landing_y_inches']-y)/(target_y-y)
    measured_apex = observed['reported_peak_height_inches']*physics.INCH
    with (physics.ROOT/'doc/physics-shooter/drag.csv').open(newline='') as f:
        table = list(csv.DictReader(f))
    distances = np.array([float(r['distance_in']) for r in table])
    nominal = {key: float(np.interp(requested_range, distances, [float(r[key]) for r in table]))
               for key in ('launch_deg', 'hood_deg', 'tps', 'exit_mps', 'vertex_cm', 'entry_deg', 'flight_s')}
    cd = .47
    d = actual_range*physics.INCH
    speed_only = float(physics.solve_speed(d, nominal['launch_deg'], cd, steps=240))
    at_range = physics.simulate(d, nominal['launch_deg'], speed_only, cd, steps=400)

    # Enumerate possible ACTUAL launch angles for diagnosis, not hood commands.
    # Values above 75 degrees are permitted here only to test the release-angle assumption.
    angles = np.linspace(55, 85, 601)
    speeds = physics.solve_speed(d, angles, cd, steps=160)
    trajectories = physics.simulate(d, angles, speeds, cd, steps=400)
    good = trajectories['valid'] & (trajectories['vz'] < 0)
    ordered = np.argsort(trajectories['vertex'][good])
    peak_axis = trajectories['vertex'][good][ordered]
    angle_axis = angles[good][ordered]
    if measured_apex < peak_axis[0] or measured_apex > peak_axis[-1]:
        raise ValueError('Observed peak is outside the diagnostic angle scan')
    inferred_angle = float(np.interp(measured_apex, peak_axis, angle_axis))
    inferred_speed = float(physics.solve_speed(d, inferred_angle, cd, steps=240))
    explanation = physics.simulate(d, inferred_angle, inferred_speed, cd, steps=400)
    report = dict(
        status='DIAGNOSTIC ONLY; does not update calibration or LUT',
        assumptions=dict(cd=cd, target_height_m=physics.HEIGHT,
                         landing_bearing_unchanged=True, landing_x_measured=False),
        requested_range_in=requested_range, observed_range_in_assuming_same_bearing=actual_range,
        overshoot_y_in=observed['reported_landing_y_inches']-target_y,
        nominal_lut=nominal,
        speed_only_explanation=dict(launch_deg=nominal['launch_deg'], exit_mps=speed_only,
                                    exit_speed_ratio=speed_only/nominal['exit_mps'],
                                    vertex_in=float(at_range['vertex']/physics.INCH),
                                    entry_deg=float(at_range['entry']), flight_s=float(at_range['time'])),
        range_and_peak_explanation=dict(launch_deg=inferred_angle, exit_mps=inferred_speed,
                                       launch_offset_deg=inferred_angle-nominal['launch_deg'],
                                       exit_speed_ratio=inferred_speed/nominal['exit_mps'],
                                       vertex_in=float(explanation['vertex']/physics.INCH),
                                       entry_deg=float(explanation['entry']), flight_s=float(explanation['time'])),
        reported_entry_deg=observed['reported_entry_angle_down_degrees_approx'],
        reported_flight_s=observed['reported_flight_time_seconds_range'],
        note='Peak/range/time/angle are approximate observations across inconsistent shots; do not fit them as one exact trajectory.'
    )
    output = physics.ROOT/'doc/physics-shooter/observation-diagnosis.json'
    output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
