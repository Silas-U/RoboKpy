import unittest
import numpy as np
from robokpy.utils import validate_keys, clamp, skew, ALLOWED_KEYS


class TestUtils(unittest.TestCase):
    def setUp(self):
        self.valid_dh = {
            "joint_name": "j1",
            "joint_type": "r",
            "link_length": 0.1,
            "twist": 0.0,
            "joint_offset": 0.0,
            "theta": 0.0,
            "offset": 0.0,
        }

    def test_validate_keys_valid(self):
        result = validate_keys(self.valid_dh)
        self.assertEqual(result, self.valid_dh)

    def test_validate_keys_missing_key(self):
        bad_dh = self.valid_dh.copy()
        bad_dh.pop("theta")
        with self.assertRaises(KeyError):
            validate_keys(bad_dh)

    def test_validate_keys_invalid_key(self):
        bad_dh = self.valid_dh.copy()
        bad_dh["bad_param"] = 123
        with self.assertRaises(KeyError):
            validate_keys(bad_dh)

    def test_clamp_within_range(self):
        self.assertEqual(clamp(5, 0, 10), 5)

    def test_clamp_below_range(self):
        self.assertEqual(clamp(-1, 0, 10), 0)

    def test_clamp_above_range(self):
        self.assertEqual(clamp(20, 0, 10), 10)

    def test_clamp_edge_case(self):
        self.assertEqual(clamp(0, 0, 10), 0)
        self.assertEqual(clamp(10, 0, 10), 10)

    def test_skew_matrix(self):
        v = [1, 2, 3]
        result = skew(v)
        expected = [[0.0, -3.0, 2.0],
                    [3.0, 0.0, -1.0],
                    [-2.0, 1.0, 0.0]]
        np.testing.assert_array_almost_equal(result, expected)

    def test_skew_with_zero_vector(self):
        v = [0, 0, 0]
        result = skew(v)
        expected = [[0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0]]
        np.testing.assert_array_almost_equal(result, expected)


if __name__ == "__main__":
    unittest.main()
