import unittest

from laser_path_planner import (
    StepperConfig, _dpy_to_step_deltas, plan_grid_path, planned_move_steps,
)


class GridPlannerTests(unittest.TestCase):
    def setUp(self):
        self.cfg = StepperConfig(
            step_angle_deg=1.8, microstep_div=16, gear_ratio=10
        )
        # A simple angular rectangle at a constant distance is sufficient to
        # verify ordering, corner inclusion and exact delta bookkeeping.
        self.corners = {
            "A": (10.0, 10.0, -10.0),
            "B": (10.0, 10.0, 10.0),
            "C": (10.0, -10.0, 10.0),
            "D": (10.0, -10.0, -10.0),
        }

    def plan(self, x_segments=2, y_segments=2):
        return plan_grid_path(
            corners=self.corners,
            x_segments=x_segments,
            y_segments=y_segments,
            stepper_pitch=self.cfg,
            stepper_yaw=self.cfg,
        )

    def test_serpentine_grid_count_and_order(self):
        result = self.plan(2, 2)
        self.assertEqual(result["plan_type"], "grid")
        self.assertEqual(result["scan_step"], 1)
        self.assertEqual(len(result["xyz"]), 9)
        self.assertEqual(
            result["grid_indices"],
            [(0, 0), (0, 1), (0, 2),
             (1, 2), (1, 1), (1, 0),
             (2, 0), (2, 1), (2, 2)],
        )
        for current, following in zip(
            result["grid_indices"], result["grid_indices"][1:]
        ):
            manhattan = abs(current[0] - following[0]) + abs(
                current[1] - following[1]
            )
            self.assertEqual(manhattan, 1)

    def test_path_starts_at_d_and_includes_all_corners(self):
        result = self.plan(3, 2)
        indices = result["grid_indices"]
        self.assertEqual(indices[0], (0, 0))
        self.assertIn((0, 2), indices)
        self.assertIn((3, 0), indices)
        self.assertIn((3, 2), indices)

    def test_step_delta_lengths_match_adjacent_targets(self):
        result = self.plan(3, 4)
        point_count = (3 + 1) * (4 + 1)
        self.assertEqual(len(result["dpy"]), point_count)
        self.assertEqual(len(result["pitch_steps_delta"]), point_count - 1)
        self.assertEqual(len(result["yaw_steps_delta"]), point_count - 1)

    def test_invalid_grid_is_rejected(self):
        with self.assertRaises(ValueError):
            self.plan(0, 2)
        invalid = dict(self.corners)
        invalid["A"] = (0.0, 0.0, 0.0)
        with self.assertRaises(ValueError):
            plan_grid_path(
                corners=invalid,
                x_segments=2,
                y_segments=2,
                stepper_pitch=self.cfg,
                stepper_yaw=self.cfg,
            )

    def test_yaw_wrap_uses_short_rotation(self):
        cfg = StepperConfig(step_angle_deg=1.0, microstep_div=1, gear_ratio=1)
        corners = {
            "A": (10.0, 5.0, 179.0),
            "B": (10.0, 5.0, -179.0),
            "C": (10.0, -5.0, -179.0),
            "D": (10.0, -5.0, 179.0),
        }
        result = plan_grid_path(
            corners=corners,
            x_segments=1,
            y_segments=1,
            stepper_pitch=cfg,
            stepper_yaw=cfg,
        )
        self.assertTrue(all(abs(delta) <= 2 for delta in result["yaw_steps_delta"]))

    def test_adjacent_path_steps_are_exactly_reversible(self):
        pitch = [10, -20]
        yaw = [30, 40]
        self.assertEqual(planned_move_steps(pitch, yaw, 0, 1), (30, 10, 0))
        self.assertEqual(planned_move_steps(pitch, yaw, 1, 0), (-30, -10, 0))
        self.assertEqual(planned_move_steps(pitch, yaw, 1, 2), (40, -20, 1))
        self.assertEqual(planned_move_steps(pitch, yaw, 2, 1), (-40, 20, 1))
        with self.assertRaises(ValueError):
            planned_move_steps(pitch, yaw, 0, 2)

    def test_absolute_quantization_prevents_rounding_drift(self):
        cfg = StepperConfig(step_angle_deg=1.0, microstep_div=1, gear_ratio=10)
        dpy = [(10.0, index * 0.04, 0.0) for index in range(26)]
        pitch_delta, yaw_delta = _dpy_to_step_deltas(dpy, cfg, cfg)
        self.assertEqual(sum(pitch_delta), 10)
        self.assertEqual(sum(yaw_delta), 0)


if __name__ == "__main__":
    unittest.main()
