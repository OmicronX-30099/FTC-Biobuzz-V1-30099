import unittest
from unittest.mock import patch

import numpy as np

import generate as physics


class PhysicsTests(unittest.TestCase):
    def test_reference_45_degree_entries_and_analytic_intersections(self):
        distances = np.array([35, 50, 60, 75])*physics.INCH
        # Preserve the attachment's 113 cm analytical reference, independently of the live target.
        with patch.object(physics, 'HEIGHT', 1.13):
            angles = np.rad2deg(np.arctan(2*physics.HEIGHT/distances+1))
            speeds = physics.ideal_speed(distances, angles)
            sim = physics.simulate(distances, angles, speeds, cd=0, steps=200)
        np.testing.assert_allclose(sim['z'], 1.13, atol=1e-9)
        np.testing.assert_allclose(sim['entry'], 45, atol=1e-9)
        np.testing.assert_allclose(angles, [74.2, 70.2, 68.1, 65.4], atol=.11)
        np.testing.assert_allclose(sim['vertex']*100, [122.8, 129.8, 134.9, 142.9], atol=.25)
        self.assertTrue(np.all(sim['vertex_x'] < distances))

    def test_drag_root_and_integration_convergence(self):
        d = np.array([35, 55, 75])*physics.INCH
        angles = np.array([75, 71, 68])
        speeds = physics.solve_speed(d, angles, .47, steps=160)
        coarse = physics.simulate(d, angles, speeds, .47, steps=160)
        fine = physics.simulate(d, angles, speeds, .47, steps=640)
        np.testing.assert_allclose(fine['z'], physics.HEIGHT, atol=2e-5)
        np.testing.assert_allclose(coarse['z'], fine['z'], atol=1e-7)
        self.assertTrue(np.all(fine['vz'] < 0))
        self.assertTrue(np.all(speeds > physics.ideal_speed(d, angles)))

    def test_ideal_short_shot_is_height_infeasible_and_maximizes_apex(self):
        with patch.object(physics, 'HEIGHT', 1.13), patch.object(physics, 'PREFERRED_ENTRY_DEG', 45):
            result = physics.optimize([35, 75], 0, .003, 1, .1)
        self.assertFalse(result[0]['feasible'])
        self.assertEqual(result[0]['hood_deg'], 15)
        self.assertAlmostEqual(result[0]['vertex_cm'], 125.8, delta=.2)
        self.assertTrue(result[1]['feasible'])
        self.assertGreaterEqual(result[1]['vertex_cm'], 136)
        self.assertAlmostEqual(result[1]['entry_deg'], 45, delta=.5)

    def test_current_target_is_104cm_and_ideal_shot_intersects_on_descent(self):
        self.assertEqual(physics.HEIGHT, 1.04)
        d = 60*physics.INCH
        angle = np.rad2deg(np.arctan(2*physics.HEIGHT/d+np.tan(np.deg2rad(physics.PREFERRED_ENTRY_DEG))))
        speed = physics.ideal_speed(d, angle)
        result = physics.simulate(d, angle, speed, 0, steps=200)
        self.assertAlmostEqual(float(result['z']), 1.04, places=9)
        self.assertAlmostEqual(float(result['entry']), 30, places=9)
        self.assertLess(float(result['vz']), 0)

    def test_30_degree_preference_does_not_trade_away_peak_height(self):
        self.assertEqual(physics.PREFERRED_ENTRY_DEG, 30)
        result = physics.optimize([75], 0, .003, 1, .1)[0]
        self.assertTrue(result['feasible'])
        self.assertGreaterEqual(result['vertex_cm'], 136)
        self.assertGreater(result['entry_deg'], 30)
        self.assertAlmostEqual(result['entry_error_deg'], result['entry_deg']-30, places=9)

    def test_unknown_angles_are_explicit_and_deleted_row_stays_out_of_fit(self):
        rows = physics.read_measurements()
        self.assertEqual(sum(r['include_in_fit'] for r in rows), 4)
        self.assertEqual(sum(r['hood_source'] == 'measured' for r in rows), 1)
        self.assertAlmostEqual(physics.HOOD_DEGREES_PER_POSITION, 64.7635135135)
        self.assertAlmostEqual(15+(.485-.16)*physics.HOOD_DEGREES_PER_POSITION, 36.0481418919)
        self.assertFalse(rows[3]['include_in_fit'])

    def test_rejects_nonphysical_speed_transfer(self):
        with self.assertRaises(ValueError):
            physics.fit_transfer([1000, 1500], [6, 5])


if __name__ == '__main__':
    unittest.main()
