"""Offline checks for continuous-trajectory bounds and interpolation."""
import unittest

import numpy as np

from draw_continuous_reference import PATHS, grid_to_joints, interpolate, validate_contact_goal


class ContinuousTests(unittest.TestCase):
    def test_interpolation_limits_every_joint_increment(self):
        start = np.array([24, 54])
        targets = list(interpolate([start, [28, 54], [28, 59], [24, 54]]))
        deltas = np.diff(np.vstack([start, targets]), axis=0)
        self.assertLessEqual(float(np.max(np.abs(deltas))), 0.3000001)
        np.testing.assert_allclose(targets[-1], [24, 54])

    def test_all_outline_vertices_are_in_local_envelope(self):
        for path in PATHS:
            for point in path:
                pan, elbow = grid_to_joints(point)
                validate_contact_goal("shoulder_pan", pan)
                validate_contact_goal("elbow_flex", elbow)

    def test_unsafe_contact_goals_are_rejected(self):
        for joint, value in [("shoulder_pan", 40), ("elbow_flex", 90),
                             ("shoulder_lift", 70), ("shoulder_pan", float("nan"))]:
            with self.subTest(joint=joint, value=value):
                with self.assertRaises(RuntimeError):
                    validate_contact_goal(joint, value)


if __name__ == "__main__":
    unittest.main()
