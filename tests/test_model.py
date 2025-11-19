import unittest
import numpy as np
from Robokpy.model import RobotModel


class TestRobotModel(unittest.TestCase):
    def setUp(self):
        self.dh_args = [
            {"joint_name": "j1", "joint_type": "r", "link_length": 0.5,
             "twist": 0.0, "joint_offset": 0.0, "theta": 0.0, "offset": 0.0},
            {"joint_name": "j2", "joint_type": "p", "link_length": 0.3,
             "twist": 90.0, "joint_offset": 0.1, "theta": 0.0, "offset": 0.0},
        ]
        self.robot = RobotModel(self.dh_args, robot_name="TestBot")

    def test_init_valid(self):
        self.assertEqual(self.robot.get_robot_name(), "TestBot")
        self.assertEqual(self.robot.get_num_of_joints(), 2)

    def test_init_invalid_empty(self):
        with self.assertRaises(ValueError):
            RobotModel([], "BadBot")

    def test_get_dh_table_shape(self):
        table = self.robot.get_dh_table()
        # Should be len(args) rows
        self.assertEqual(table.shape[0], len(self.dh_args))

    def test_get_joint_type(self):
        jt = self.robot.get_joint_type()
        self.assertEqual(jt, ["r", "p"])

    def test_get_joint_names(self):
        names = self.robot.get_joint_names()
        self.assertEqual(names, ["j1", "j2"])

    def test_sum_link_lengths(self):
        total = self.robot.get_sum_link_lengths()
        self.assertAlmostEqual(total, 0.8)

    def test_joint_limits(self):
        
        expected_limits = (
            np.array([0, 0]),
            np.array([np.pi, 1.0])
        )

        self.robot.set_joint_limits(
            {
                "min": { "j1": 0, "j2": 0},
                "max": { "j1": np.pi,"j2": 1.0}
            }
        )
        
        jl = self.robot.get_joint_limits()
        self.assertTrue(np.allclose(jl[0], expected_limits[0]))
        self.assertTrue(np.allclose(jl[1], expected_limits[1]))

    def test_set_joint_limits_mismatch(self):
        with self.assertRaises(ValueError):
            self.robot.set_joint_limits([(0, np.pi)])  # wrong length

    def test_structure_print(self):
        # Should print "RP" (uppercase joint types)
        # Capture printed output
        import io, sys
        captured_output = io.StringIO()
        sys.stdout = captured_output
        self.robot.structure()
        sys.stdout = sys.__stdout__
        output = captured_output.getvalue().strip()
        self.assertEqual(output, "RP")


if __name__ == "__main__":
    unittest.main()
