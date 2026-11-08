import sys
import unittest
from pathlib import Path

import numpy as np

PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "src" / "safe_obstacle_avoidance"
sys.path.insert(0, str(PACKAGE_ROOT))

from safe_obstacle_avoidance.geometry import depth_image_to_ellipses


class TestDepthImageToEllipses(unittest.TestCase):
    def test_returns_finite_ellipse_rows(self):
        image = np.full((240, 320), 2.0, dtype=np.float32)
        ellipses = depth_image_to_ellipses(image)
        self.assertEqual(ellipses.ndim, 2)
        self.assertEqual(ellipses.shape[1], 5)
        self.assertGreater(ellipses.shape[0], 100)
        self.assertTrue(np.all(np.isfinite(ellipses)))
        self.assertTrue(np.all(ellipses[:, 2] > 0.0))
        self.assertTrue(np.all(ellipses[:, 3] > 0.0))

    def test_forward_ray_is_centered_on_local_positive_y(self):
        image = np.full((120, 160), 1.5, dtype=np.float32)
        ellipses = depth_image_to_ellipses(image)
        closest = int(np.argmin(np.abs(ellipses[:, 4] - np.pi / 2.0)))
        row = ellipses[closest]
        self.assertAlmostEqual(row[0], 0.0, delta=0.10)
        self.assertGreater(row[1], 0.0)
        self.assertAlmostEqual(row[4], np.pi / 2.0, delta=0.05)

    def test_invalid_depth_is_replaced_conservatively(self):
        image = np.full((80, 160), np.nan, dtype=np.float32)
        ellipses = depth_image_to_ellipses(image)
        self.assertTrue(np.all(np.isfinite(ellipses)))

    def test_bad_image_shape_is_rejected(self):
        with self.assertRaises(ValueError):
            depth_image_to_ellipses(np.zeros((20, 20, 3), dtype=np.float32))


if __name__ == "__main__":
    unittest.main()
