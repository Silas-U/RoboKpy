import unittest
from robokpy import Init_Model
from Model import DHModel
import matplotlib
matplotlib.use("Agg") 


class TestJacobian(unittest.TestCase):
    def setUp(self):
        robot_model = DHModel.get_model("Puma561")
        self.rb = Init_Model(robot_model, robot_name="Puma561", twist_in_rads=False)

    def test_compute_shape(self):
        self.rb.fk.compute([0, 0.7854, 3.1416, 0, 0.7854, 0], rads=True) 
        J = self.rb.jac.compute()
        # Should be 6 x n
        self.assertEqual(J.shape, (6, self.rb.model.num_of_joints))

    def test_jac_rank_full(self):
        self.rb.fk.compute([0, 0.7854, 3.1416, 0, 0.7854, 0], rads=True) 
        self.rb.jac.compute()
        rank = self.rb.jac.rank()
        self.assertEqual(rank, self.rb.model.num_of_joints)

    def test_check_singularity_no_singularity(self):
        self.rb.fk.compute([0, 0.7854, 3.1416, 0, 0.7854, 0], rads=True) 
        self.rb.jac.compute()
        # No error should be raised here
        self.rb.jac.check_singularity()

    def test_check_singularity_detected(self):
        self.rb.fk.compute([0, 0.7854, 3.1416, 0, 0.7854, 0], rads=True) 
        # Model modified to have prismatic joints (reduces rank artificially)
        self.rb.model.joint_type_info = ["p"] * self.rb.model.num_of_joints
        self.rb.jac.compute()
        rank = self.rb.jac.rank()
        print(rank)
        print(self.rb.model.num_of_joints)
        self.assertLess(rank, self.rb.model.num_of_joints)


if __name__ == "__main__":
    unittest.main()
