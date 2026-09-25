"""Failure-path checks that require no hardware."""
import json
import pathlib
import tempfile
import types
import unittest
from unittest.mock import Mock, patch

from draw_reference import Arm, save_json


class CheckpointTests(unittest.TestCase):
    @patch("draw_reference.os.fsync")
    def test_checkpoint_syncs_file_and_directory(self, fsync):
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "state.json"
            save_json(path, {"next": 94})
            self.assertEqual(json.loads(path.read_text()), {"next": 94})
            self.assertFalse(path.with_suffix(".tmp").exists())
        self.assertEqual(fsync.call_count, 2)

    def test_failed_file_sync_preserves_previous_checkpoint(self):
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "state.json"
            save_json(path, {"next": 94})
            with patch("draw_reference.os.fsync", side_effect=OSError("sync failed")):
                with self.assertRaises(OSError):
                    save_json(path, {"next": 95})
            self.assertEqual(json.loads(path.read_text()), {"next": 94})


class SafetyTests(unittest.TestCase):
    def arm(self, position=55.0, load=30):
        arm = Arm.__new__(Arm)
        arm.clear = True
        arm.offsets = {}
        arm.move = Mock()
        arm.position = Mock(return_value=position)
        arm.load = Mock(return_value=load)
        arm.log = Mock()
        arm.write = Mock()
        arm.bus = types.SimpleNamespace(calibration={
            joint: types.SimpleNamespace(range_min=930, range_max=3298)
            for joint in ("shoulder_lift", "shoulder_pan", "elbow_flex", "wrist_flex", "wrist_roll")
        })
        return arm

    @patch("draw_reference.time.sleep")
    def test_failed_lift_cannot_authorize_lateral_move(self, sleep):
        arm = self.arm(position=59)
        with self.assertRaisesRegex(RuntimeError, "Lift not clear"):
            arm.lift(60)
        self.assertFalse(arm.clear)
        with self.assertRaisesRegex(RuntimeError, "verified lift"):
            arm.target("shoulder_pan", 54)
        arm.write.assert_not_called()

    @patch("draw_reference.time.sleep")
    def test_remaining_contact_load_blocks_lift(self, sleep):
        arm = self.arm(position=54, load=600)
        with self.assertRaisesRegex(RuntimeError, "Lift not clear"):
            arm.lift(60)
        self.assertFalse(arm.clear)

    def test_verified_bidirectional_row_axis_can_increase_when_clear(self):
        arm = self.arm(position=54)
        arm.target("wrist_roll", 56)
        arm.write.assert_called_once_with("Goal_Position", "wrist_roll", 2751, raw=True)

    def test_all_non_lift_joints_require_clearance(self):
        arm = self.arm()
        arm.clear = False
        for joint in ("shoulder_pan", "elbow_flex", "wrist_flex", "wrist_roll"):
            with self.subTest(joint=joint):
                with self.assertRaisesRegex(RuntimeError, "verified lift"):
                    arm.target(joint, 54)
        arm.write.assert_not_called()

    def test_limits_checked_before_a_position_write(self):
        arm = self.arm()
        with self.assertRaisesRegex(RuntimeError, "Unsafe"):
            arm.target("shoulder_lift", 180)
        arm.write.assert_not_called()

    def test_negative_decoded_load_is_not_an_overload(self):
        arm = self.arm()
        arm.read = Mock(return_value=-20)
        self.assertEqual(Arm.load(arm, "shoulder_pan"), 20)

    def test_move_timeout_is_failure(self):
        arm = self.arm()
        with self.assertRaisesRegex(RuntimeError, "Move timeout"):
            Arm.move(arm, "shoulder_lift", 54, timeout=0)

    @patch("draw_reference.time.sleep")
    def test_settled_gravity_offset_is_compensated(self, sleep):
        arm = self.arm()
        actual = [60.0]
        arm.position = lambda joint: actual[0]
        arm.health = Mock()
        arm.read = Mock(return_value=0)
        arm.target = lambda joint, command: actual.__setitem__(0, command + 1.5)
        reached = Arm.move(arm, "shoulder_lift", 54, timeout=1)
        self.assertLess(abs(reached - 54), 0.05)


if __name__ == "__main__":
    unittest.main()
