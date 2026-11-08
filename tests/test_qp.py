import sys
import unittest
from pathlib import Path

import numpy as np

PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "src" / "safe_obstacle_avoidance"
sys.path.insert(0, str(PACKAGE_ROOT))

from safe_obstacle_avoidance.qp import solve_2d_qp


class TestSolve2dQp(unittest.TestCase):
    def test_origin_is_optimal_when_feasible(self):
        G = np.array([[1.0, 0.0], [-1.0, 0.0], [0.0, 1.0], [0.0, -1.0]])
        h = np.ones(4)
        result = solve_2d_qp((2.0, 1.0), G, h)
        np.testing.assert_allclose(result, [0.0, 0.0], atol=1e-9)

    def test_projection_to_single_active_constraint(self):
        G = np.array([[1.0, 0.0], [-1.0, 0.0], [0.0, 1.0], [0.0, -1.0]])
        h = np.array([-1.0, 3.0, 2.0, 2.0])
        result = solve_2d_qp((1.0, 1.0), G, h)
        np.testing.assert_allclose(result, [-1.0, 0.0], atol=1e-8)

    def test_vertex_solution(self):
        # x <= -1 and y <= -2 force the closest feasible vertex (-1, -2).
        G = np.array([[1.0, 0.0], [0.0, 1.0], [-1.0, 0.0], [0.0, -1.0]])
        h = np.array([-1.0, -2.0, 3.0, 3.0])
        result = solve_2d_qp((1.0, 1.0), G, h)
        np.testing.assert_allclose(result, [-1.0, -2.0], atol=1e-8)

    def test_infeasible_constraints_return_none(self):
        G = np.array([[1.0, 0.0], [-1.0, 0.0]])
        h = np.array([-1.0, -1.0])
        self.assertIsNone(solve_2d_qp((1.0, 1.0), G, h))

    def test_invalid_q_is_rejected(self):
        with self.assertRaises(ValueError):
            solve_2d_qp((0.0, 1.0), np.zeros((0, 2)), np.zeros(0))


if __name__ == "__main__":
    unittest.main()
