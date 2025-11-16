import unittest
import numpy as np
import matplotlib
matplotlib.use("Agg")
from robokpy.mviz import VizModel
from Models.Model import DHModel
from robokpy.model import RobotModel
from robokpy.fk import ForwardKinematics


class TestVizModel(unittest.TestCase):
    def setUp(self):
        self.robot_model = DHModel.get_model("Puma561")
        self.model = RobotModel(self.robot_model, robot_name="TestBot")
        self.fk = ForwardKinematics(self.model)
        self.viz = VizModel(self.model, self.fk)

    def test_init_sets_attributes(self):
        self.assertEqual(self.viz.robot_instance.get_robot_name(), "TestBot")
        self.assertEqual(self.viz.background_color, "w")

    def test_rotation_matrix(self):
        R = self.viz.rotation_matrix("z", 90)
        self.assertTrue(np.allclose(R.shape, (3,3)))

    def test_plot_coordinate_frame_returns_quivers(self):
        origin = np.zeros(3)
        Rmat = np.eye(3)
        quivers = self.viz.plot_coordinate_frame(self.viz.ax, origin, Rmat, axis_length=0.1)
        self.assertEqual(len(quivers), 3)

    def test_show_target_runs(self):
        tar = [np.array([0.1, 0.2, 0.3, 0.0, 0.0, 0.0])]
        self.viz.show_target(tar, axis_length=0.05, sc=True)

    def test_init_clears_plot(self):
        self.viz.init()
        # after init, lines must have empty data
        x, y = self.viz.arm_line.get_data()
        self.assertEqual(len(x), 0)

    def test_scale_viz_updates_limits(self):
        self.viz.scale_viz(0.5, fram_axis_len=0.1)
        xlim = self.viz.ax.get_xlim()
        self.assertAlmostEqual(xlim[0], -0.5)
        self.assertAlmostEqual(xlim[1], 0.5)

    def test_show_dh_model_invalid_length(self):
        with self.assertRaises(ValueError):
            self.viz.show_dh_model(joints_v=[0.1, 0.2])

    def test_show_dh_model_type_error(self):
        with self.assertRaises(TypeError):
            self.viz.show_dh_model(joints_v=["bad_input", 0, 0, 0, 0, 0])

if __name__ == "__main__":
    unittest.main()
