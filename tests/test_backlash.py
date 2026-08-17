import unittest

from backlash import (
    BacklashAxisState,
    DEFAULT_BACKLASH_STEPS,
    final_approach_source,
)


class BacklashAxisStateTests(unittest.TestCase):
    def test_project_calibration_values(self):
        self.assertEqual(DEFAULT_BACKLASH_STEPS, {"x": 77, "y": 4})

    def test_final_approach_source_finishes_in_recorded_direction(self):
        self.assertEqual(final_approach_source(1000, +1, 77, 10), 913)
        self.assertEqual(final_approach_source(1000, -1, 77, 10), 1087)

    def test_final_approach_source_validates_inputs(self):
        with self.assertRaises(ValueError):
            final_approach_source(0, 0, 77)
        with self.assertRaises(ValueError):
            final_approach_source(0, 1, -1)

    def test_two_stage_move_ends_on_requested_flank(self):
        for direction, expected_gap in ((-1, 0), (+1, 77)):
            state = BacklashAxisState(77)
            state.initialize(-direction, logical_position_steps=250)
            source = final_approach_source(1000, direction, 77, 10)
            state.apply_pulses(state.pulses_to_target(source))
            state.apply_pulses(state.pulses_to_target(1000))

            self.assertEqual(state.logical_position_steps, 1000)
            self.assertEqual(state.gap_steps, expected_gap)

    def test_state_is_unknown_until_preloaded(self):
        state = BacklashAxisState(40)
        self.assertFalse(state.initialized)
        self.assertIsNone(state.logical_position_steps)
        self.assertIsNone(state.gap_steps)
        with self.assertRaises(RuntimeError):
            state.apply_pulses(1)
        with self.assertRaises(RuntimeError):
            state.pulses_to_target(10)

    def test_positive_and_negative_preload_select_the_correct_flank(self):
        positive = BacklashAxisState(40)
        positive.initialize(+1, logical_position_steps=12)
        self.assertEqual(positive.logical_position_steps, 12)
        self.assertEqual(positive.gap_steps, 40)

        negative = BacklashAxisState(40)
        negative.initialize(-1, logical_position_steps=-8)
        self.assertEqual(negative.logical_position_steps, -8)
        self.assertEqual(negative.gap_steps, 0)

    def test_manual_tracking_matches_selected_example(self):
        state = BacklashAxisState(40)
        state.initialize(+1)

        self.assertEqual(state.apply_pulses(+5), +5)
        self.assertEqual(state.logical_position_steps, 5)
        self.assertEqual(state.gap_steps, 40)

        self.assertEqual(state.apply_pulses(-5), 0)
        self.assertEqual(state.logical_position_steps, 5)
        self.assertEqual(state.gap_steps, 35)

        self.assertEqual(state.apply_pulses(+10), +5)
        self.assertEqual(state.logical_position_steps, 10)
        self.assertEqual(state.gap_steps, 40)

    def test_manual_tracking_is_symmetric_from_negative_flank(self):
        state = BacklashAxisState(40)
        state.initialize(-1)

        self.assertEqual(state.apply_pulses(-5), -5)
        self.assertEqual(state.logical_position_steps, -5)
        self.assertEqual(state.gap_steps, 0)

        self.assertEqual(state.apply_pulses(+5), 0)
        self.assertEqual(state.logical_position_steps, -5)
        self.assertEqual(state.gap_steps, 5)

        self.assertEqual(state.apply_pulses(-10), -5)
        self.assertEqual(state.logical_position_steps, -10)
        self.assertEqual(state.gap_steps, 0)

    def test_positive_auto_target_adds_remaining_clearance(self):
        state = BacklashAxisState(40)
        state.initialize(+1, logical_position_steps=1000)
        state.apply_pulses(-25)
        self.assertEqual(state.gap_steps, 15)

        command = state.pulses_to_target(1300)
        self.assertEqual(command, 325)
        self.assertEqual(state.apply_pulses(command), 300)
        self.assertEqual(state.logical_position_steps, 1300)
        self.assertEqual(state.gap_steps, 40)

    def test_negative_auto_target_adds_remaining_clearance(self):
        state = BacklashAxisState(40)
        state.initialize(+1, logical_position_steps=1000)
        state.apply_pulses(-25)
        self.assertEqual(state.gap_steps, 15)

        command = state.pulses_to_target(700)
        self.assertEqual(command, -315)
        self.assertEqual(state.apply_pulses(command), -300)
        self.assertEqual(state.logical_position_steps, 700)
        self.assertEqual(state.gap_steps, 0)

    def test_target_equal_to_position_does_not_change_gap(self):
        state = BacklashAxisState(40)
        state.initialize(+1, logical_position_steps=100)
        state.apply_pulses(-10)
        self.assertEqual(state.gap_steps, 30)

        self.assertEqual(state.pulses_to_target(100), 0)
        self.assertEqual(state.gap_steps, 30)

    def test_cancelled_move_can_be_replanned_from_actual_emitted_pulses(self):
        state = BacklashAxisState(40)
        state.initialize(+1, logical_position_steps=1000)
        state.apply_pulses(-25)

        self.assertEqual(state.pulses_to_target(1300), 325)
        self.assertEqual(state.apply_pulses(20), 0)
        self.assertEqual(state.logical_position_steps, 1000)
        self.assertEqual(state.gap_steps, 35)
        self.assertEqual(state.pulses_to_target(1300), 305)

    def test_zero_backlash_matches_raw_pulse_position(self):
        state = BacklashAxisState(0)
        state.initialize(+1)
        self.assertEqual(state.apply_pulses(17), 17)
        self.assertEqual(state.apply_pulses(-9), -9)
        self.assertEqual(state.logical_position_steps, 8)
        self.assertEqual(state.pulses_to_target(-2), -10)

    def test_invalidate_discards_position_and_gap(self):
        state = BacklashAxisState(4)
        state.initialize(+1)
        state.apply_pulses(10)
        state.invalidate()
        self.assertFalse(state.initialized)
        self.assertIsNone(state.logical_position_steps)
        self.assertIsNone(state.gap_steps)

    def test_invalid_configuration_and_direction_are_rejected(self):
        with self.assertRaises(ValueError):
            BacklashAxisState(-1)
        state = BacklashAxisState(10)
        with self.assertRaises(ValueError):
            state.initialize(0)


if __name__ == "__main__":
    unittest.main()
