import unittest
import numpy as np
from robokpy import Init_Model
from Model import DHModel
import matplotlib
matplotlib.use("Agg")


class TestInverseKinematics(unittest.TestCase):
    def setUp(self):
        robot_model = DHModel.get_model("Puma561")
        self.rb = Init_Model(robot_model, robot_name="Puma561", twist_in_rads=False)

    def test_solve_returns_list(self):
        self.rb.fk.compute([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) 
        target = self.rb.fk.get_target()
        result = self.rb.ik.solve(target, tol=1e-2, max_iter=10)
        self.assertIsInstance(result, list)
        self.assertEqual(len(result), self.rb.model.num_of_joints)

    def test_solve_converges(self):
        self.rb.fk.compute([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) 
        target = self.rb.fk.get_target()
        result = self.rb.ik.solve(target, tol=1e-2, max_iter=10)
        self.assertTrue(np.allclose(result, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))

    def test_solve_nonconvergence(self):
        self.rb.fk.compute([100.0, 0.0, 0.0, 0.0, 0.0, 0.0]) 
        target = self.rb.fk.get_target()
        result = self.rb.ik.solve(target, tol=1e-12, max_iter=2)
        self.assertEqual(len(result), self.rb.model.num_of_joints)


if __name__ == "__main__":
    unittest.main()
