"""Offline bounds for the new contact experiment controller."""
import unittest
from unittest.mock import Mock

from contact_lab import Lab, validate_pose


class ContactLabTests(unittest.TestCase):
    def test_local_bounds_reject_unsafe_and_unknown_targets(self):
        for target in ({"gripper": 0}, {"elbow_flex": 90},
                       {"shoulder_lift": float("nan")}, {"wrist_flex": -102}):
            with self.subTest(target=target):
                with self.assertRaises(ValueError):
                    validate_pose(target)

    def test_no_contact_stroke_without_contact(self):
        arm = Mock()
        with self.assertRaises(ValueError):
            Lab(arm).line(1, 0)
        arm.bus.sync_write.assert_not_called()

    def test_stroke_size_is_checked_before_writes(self):
        arm = Mock()
        lab = Lab(arm)
        lab.contact = 54
        for pan, elbow in ((4, 0), (0, -4), (float("nan"), 0)):
            with self.assertRaises(ValueError):
                lab.line(pan, elbow)
        arm.bus.sync_write.assert_not_called()

    def test_unverified_clearance_is_rejected(self):
        arm = Mock(clear=False)
        with self.assertRaises(RuntimeError):
            Lab(arm).clearance()

    def test_contact_threshold_bound_checked_before_motion(self):
        arm = Mock()
        with self.assertRaises(ValueError):
            Lab(arm).touch(54, -100)
        arm.move.assert_not_called()


if __name__ == "__main__":
    unittest.main()
