"""Offline geometry checks; no serial ports or cameras are opened."""
import importlib.util
import unittest

import numpy as np


@unittest.skipUnless(importlib.util.find_spec("mujoco"), "MuJoCo optional dependency not installed")
class RehearsalTests(unittest.TestCase):
    def setUp(self):
        from rehearse_reference import Rehearsal
        ranges = {"shoulder_pan": (873, 2932), "shoulder_lift": (930, 3298),
                  "elbow_flex": (755, 2983), "wrist_flex": (969, 3291),
                  "wrist_roll": (0, 4095)}
        calibration = {name: {"range_min": lo, "range_max": hi}
                       for name, (lo, hi) in ranges.items()}
        self.sim = Rehearsal(calibration=calibration)
        self.pose = np.array([20, 62.4, 44.13, -99.6, -71.34])

    def test_jacobian_is_in_metres_per_calibrated_degree(self):
        jac = self.sim.jacobian(self.pose)
        for index in range(5):
            delta = np.zeros(5)
            delta[index] = 0.001
            measured = (self.sim.forward(self.pose + delta) - self.sim.forward(self.pose - delta)) / 0.002
            np.testing.assert_allclose(jac[:, index], measured, atol=1e-8)

    def test_planar_ik_preserves_held_wrist(self):
        target = self.sim.forward(self.pose) + [-0.006, 0.006, 0]
        solved = self.sim.solve(target, self.pose)
        self.assertLess(np.linalg.norm(self.sim.forward(solved) - target), 1e-5)
        np.testing.assert_array_equal(solved[3:], self.pose[3:])
        self.assertTrue(np.all(solved >= self.sim.limits[:, 0]))
        self.assertTrue(np.all(solved <= self.sim.limits[:, 1]))

    def test_unreachable_target_fails_instead_of_returning_unsafe_pose(self):
        with self.assertRaisesRegex(RuntimeError, "did not converge"):
            self.sim.solve([10, 10, 10], self.pose)

    def test_nonfinite_and_out_of_envelope_seeds_are_rejected(self):
        with self.assertRaises(ValueError):
            self.sim.forward([np.nan] * 5)
        with self.assertRaisesRegex(ValueError, "outside"):
            self.sim.solve([0, 0, 0], [1000] * 5)

    def test_height_compensation_keeps_tip_on_nominal_plane(self):
        initial = self.sim.forward(self.pose)
        pose = self.pose.copy()
        pose[2] += 3
        _, achieved = self.sim.surface_point(pose, initial[2])
        self.assertLess(abs(achieved[2] - initial[2]), 1e-6)
        self.assertGreater(np.linalg.norm(achieved[:2] - initial[:2]), 0.003)


if __name__ == "__main__":
    unittest.main()
