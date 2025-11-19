import unittest
import numpy as np
from Robokpy.trajectory import TrajectoryPlanner
from Robokpy.model import RobotModel
from Robokpy.fk import ForwardKinematics
from Robokpy.ik import InverseKinematics
from Robokpy.jacobian import Jacobian


class TestTrajectoryPlanner(unittest.TestCase):

    def setUp(self):
        self.dh_args = [
            {"joint_name": "j1", "joint_type": "r", "link_length": 0.4,
             "twist": 0.0, "joint_offset": 0.2, "theta": 0.0, "offset": 0.0},
            {"joint_name": "j2", "joint_type": "r", "link_length": 0.4,
             "twist": 0.0, "joint_offset": 0.0, "theta": 0.0, "offset": 0.0},
        ]
        self.model = RobotModel(self.dh_args, robot_name="TestBot")
        self.fk = ForwardKinematics(self.model)
        self.jac = Jacobian(self.model, self.fk)
        self.ik = InverseKinematics(self.model, self.fk, self.jac)
        self.tp =TrajectoryPlanner(self.model, self.fk, self.ik, self.jac)
    
        self.tp.set_traj_time(5.0)

    def test_cubic_segment(self):
        coeff = self.tp.cubic_segment(q0=0, q1=1, v0=0, v1=0)
        self.assertEqual(len(coeff), 4)
        q, qd, qdd = self.tp.eval_cubic(coeff, t=0.5)
        self.assertTrue(isinstance(q, float))

    def test_quintic_segment(self):
        coeff = self.tp.quintic_segment(q0=0, q1=1, v0=0, v1=0, a0=0, a1=0)
        self.assertEqual(len(coeff), 6)
        q, qd, qdd = self.tp.eval_quintic(coeff, t=0.5)
        self.assertTrue(isinstance(q, float))

    def test_compute_velocities_ts(self):
        waypoints = np.array([
            [0.2, 0.3, 0.4],
            [0.2, 0.3, 0.4],   # identical (pause)
            [0.5, 0.3, 0.2]
        ]).T  # shape (3, N)

        t, v = self.tp.compute_velocities_ts(waypoints)

        self.assertEqual(len(t), waypoints.shape[1])
        self.assertEqual(v.shape, waypoints.shape[:2])
        self.assertGreater(t[1], t[0])   # pause shift

    def test_compute_velocities_js(self):
        jnts = np.array([
            [0, 0.5, 1.0],
            [0, 0.5, 1.0],
            [0, 0.5, 1.0],
            [0, 0.5, 1.0],
            [0, 0.5, 1.0],
            [0, 0.5, 1.0]
        ])  # 6x3

        t, v = self.tp.compute_velocities_js(jnts)

        self.assertEqual(v.shape, jnts.shape)
        self.assertTrue(np.all(np.isfinite(v)))

    def test_jointspace_trajectory(self):
        waypoints = [
            [0.3,  0.2,  0.2,  0.,  0.,  0.],
            [0.3,  0.02, 0.2,  0.,  0.,  0.],
            [0.3,  0.2,  0.2,  0.,  0.,  0.],
        ]

        self.tp.traj_type('qu')
        traj = self.tp.create_traj_jointspace(waypoints, n_samples=10, xyz_mask=[1, 1, 1, 0, 0, 0])

        out = traj[0]
        self.assertEqual(out.shape[1], 2)  # 6 DOF
        self.assertGreater(out.shape[0], len(waypoints))  # interpolated samples

    def test_spline_js(self):
        joint_angles = [
            [0, 0],
            [0.5, 0.1],
            [0.2, 0.4],
            [0, 0]
        ]

        self.tp.traj_type('spl')
        q = self.tp.q_spline_js(joint_angles, time_step=100)
        self.assertEqual(q.shape[1], 2)
        self.assertEqual(len(q), 100)

    def test_spline_taskspace(self):
        waypoints = [
            [0.2, 0.1, 0.3, 0, 0, 0],
            [0.3, 0.2, 0.2, 0, 0, 0],
            [0.25, 0.15, 0.25, 0, 0, 0],
            [0.2, 0.1, 0.3, 0, 0, 0]
        ]

        self.tp.traj_type('spl')
        q = self.tp.q_spline(waypoints, time_step=100)

        self.assertEqual(q.shape, (100, 3))
        self.assertTrue(np.all(np.isfinite(q)))


if __name__ == "__main__":
    unittest.main()
