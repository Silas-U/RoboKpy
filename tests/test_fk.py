import unittest
import numpy as np
from Robokpy import Init_Model
from Model import DHModel
import matplotlib
matplotlib.use("Agg") 


class TestForwardKinematics(unittest.TestCase):
    def setUp(self):
        robot_model = DHModel.get_model("Puma561")
        self.rb = Init_Model(robot_model, robot_name="Puma561", twist_in_rads=False)

    def test_group_dh(self):
        dh_params = self.rb.fk._group_dh([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], rads=True)
        self.assertEqual(len(dh_params), 6)
        self.assertEqual(self.rb.model.num_of_joints, 6)

    def test_compute(self):
        t_matrices = self.rb.fk.compute([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], rads=True)
        self.assertIsInstance(t_matrices, list)
        self.assertTrue(all(isinstance(T, np.ndarray) for T in t_matrices))

    def test_get_tcp(self):
        self.rb.fk.compute([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], rads=True)
        tcp = self.rb.fk.get_tcp()
        self.assertEqual(len(tcp), 3)
        self.assertTrue(np.all(np.isfinite(tcp)))

    def test_get_transform_valid(self):
        self.rb.fk.compute([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], rads=True)
        T = self.rb.fk.get_transform(stop_index=1)
        self.assertEqual(T.shape, (4, 4))

    def test_get_tranformations(self):
        self.rb.fk.compute([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], rads=True)
        transforms = self.rb.fk.get_tranformations()
        self.assertTrue(transforms.shape[1:] == (4, 4))

    def test_se3_output(self):
        self.rb.fk.compute([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], rads=True)
        T = self.rb.fk.get_transform(stop_index=1)
        se3 = self.rb.fk.SE3(T, deg=True, merge_res=False)
        self.assertEqual(se3.shape, (2, 3))  # position + euler

    def test_get_joint_states(self):
        self.rb.fk.compute([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], rads=True)
        joint_states_rad = self.rb.fk.get_joint_states(in_degrees=False)
        joint_states_deg = self.rb.fk.get_joint_states(in_degrees=True)
        self.assertEqual(len(joint_states_rad), 6)
        self.assertEqual(len(joint_states_deg), 6)


if __name__ == "__main__":
    unittest.main()
