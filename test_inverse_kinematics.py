
import unittest
from inverse_kinematics import inverse_kinematics


class TestInverseKinematics(unittest.TestCase):

    def test_reachable_target(self):
        result = inverse_kinematics(10, 0, 15)
        self.assertIsNotNone(result)
        self.assertTrue(result["reachable"])

    def test_unreachable_target(self):
        result = inverse_kinematics(100, 0, 15)
        self.assertIsNone(result)

    def test_invalid_coordinates(self):
        with self.assertRaises(ValueError):
            inverse_kinematics(float("nan"), 0, 15)

    def test_boundary_reachable_target(self):
        result = inverse_kinematics(20, 0, 5)
        self.assertIsNotNone(result)
        self.assertTrue(result["reachable"])

    def test_elbow_up_configuration(self):
        result = inverse_kinematics(10, 0, 15, elbow_up=True)
        self.assertIsNotNone(result)
        self.assertTrue(result["reachable"])
        self.assertLess(result["elbow"], 0)

    def test_positive_infinity_coordinates(self):
        with self.assertRaises(ValueError):
            inverse_kinematics(float("inf"), 0, 15)

    def test_negative_infinity_coordinates(self):
        with self.assertRaises(ValueError):
            inverse_kinematics(0, float("-inf"), 15)


if __name__ == "__main__":
    unittest.main()
