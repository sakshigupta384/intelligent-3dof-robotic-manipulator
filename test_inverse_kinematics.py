
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


if __name__ == "__main__":
    unittest.main()
