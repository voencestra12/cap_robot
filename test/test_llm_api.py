import unittest

from cap_robot.llm_api import (
    DEFAULT_COOPERATIVE_ACTIONS,
    DEFAULT_PNP_ACTIONS,
    get_capability_catalog,
    validate_actions,
    validate_cooperative_actions,
)


class LlmApiValidationTest(unittest.TestCase):
    def test_default_pnp_is_valid(self):
        actions = validate_actions(DEFAULT_PNP_ACTIONS, 40.0)
        self.assertEqual(len(actions), len(DEFAULT_PNP_ACTIONS))

    def test_default_cooperative_plan_is_valid(self):
        actions = validate_cooperative_actions(DEFAULT_COOPERATIVE_ACTIONS)
        self.assertEqual(len(actions), len(DEFAULT_COOPERATIVE_ACTIONS))

    def test_unknown_dynamic_api_is_rejected(self):
        actions = list(DEFAULT_COOPERATIVE_ACTIONS)
        actions[4] = {'api': 'pick_dynamic', 'target_name': 'nearest_fruit'}
        with self.assertRaises(ValueError):
            validate_cooperative_actions(actions)

    def test_cooperative_move_before_grasp_is_rejected(self):
        actions = [
            {'api': 'control_gripper', 'position': 850},
            {
                'api': 'cooperative_move_relative',
                'x_offset': 0,
                'y_offset': 0,
                'z_offset': 100,
                'speed': 40,
            },
        ]
        with self.assertRaises(ValueError):
            validate_cooperative_actions(actions)

    def test_cooperative_capabilities_are_registered(self):
        catalog = get_capability_catalog()
        self.assertIn('dual_arm_grasp', catalog)
        self.assertIn('cooperative_transport', catalog)
        self.assertIn('synchronized_release', catalog)


if __name__ == '__main__':
    unittest.main()
