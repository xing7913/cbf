"""Pure geometry helpers for the depth-image ellipsoid safe-set pipeline."""

from __future__ import annotations

import math

import cv2
import numpy as np


def depth_image_to_ellipses(
    depth_image_m: np.ndarray,
    *,
    theta_min_deg: float = 30.0,
    theta_max_deg: float = 150.0,
    r_max_ellipse: float = 5.0,
    depth_val_outside_fov: float = 0.8,
    ellipse_buffer: float = 0.2,
    ellipse_semi_minor: float = 0.15,
    num_ellipses: int = 200,
    depth_row_fraction: float = 0.60,
    min_depth: float = 0.2,
) -> np.ndarray:
    """Convert one metric depth image into rows [cx, cy, a, b, yaw].

    The ellipse convention follows the source formulation: angles are measured
    in the local plane with the camera's forward ray at pi/2 (+local Y).
    ``depth_image_m`` contains axial camera depth; it is converted to radial
    range by dividing by sin(theta), as in the source's computed radial array.
    """
    image = np.asarray(depth_image_m, dtype=np.float32)
    if image.ndim != 2 or image.size == 0:
        raise ValueError(f"Expected a non-empty 2-D depth image, got {image.shape}")
    if num_ellipses < 8:
        raise ValueError("num_ellipses must be at least 8")
    if not (0.0 < theta_min_deg < theta_max_deg < 180.0):
        raise ValueError("Require 0 < theta_min_deg < theta_max_deg < 180")
    if r_max_ellipse <= min_depth or min_depth <= 0.0:
        raise ValueError("Require r_max_ellipse > min_depth > 0")
    if ellipse_buffer <= 0.0 or ellipse_semi_minor <= 0.0:
        raise ValueError("Ellipse buffer and semi-minor axis must be positive")

    theta_min = math.radians(theta_min_deg)
    theta_max = math.radians(theta_max_deg)
    fov = theta_max - theta_min
    count_in_fov = max(2, int(num_ellipses * fov / (2.0 * math.pi)))

    height, width = image.shape
    reduced_height = max(1, int(round(count_in_fov * height / max(width, 1))))
    reduced = cv2.resize(
        image,
        (count_in_fov, reduced_height),
        interpolation=cv2.INTER_NEAREST,
    )
    row_fraction = float(np.clip(depth_row_fraction, 0.0, 1.0))
    row_index = min(int(row_fraction * reduced.shape[0]), reduced.shape[0] - 1)
    axial_depth = reduced[row_index, :].astype(np.float64)
    axial_depth = np.nan_to_num(
        axial_depth, nan=min_depth, posinf=r_max_ellipse, neginf=min_depth
    )
    axial_depth[axial_depth <= 0.0] = min_depth
    axial_depth = np.clip(axial_depth, min_depth, r_max_ellipse)

    # Image columns span left-to-right; theta=pi/2 is the forward direction.
    angles_descending = np.linspace(theta_max, theta_min, axial_depth.size)
    sin_theta = np.maximum(np.abs(np.sin(angles_descending)), 1e-6)
    radial_depth = np.clip(axial_depth / sin_theta, min_depth, r_max_ellipse)

    count_left = max(1, int((theta_min / fov) * axial_depth.size))
    count_right = max(1, int(((2.0 * math.pi - theta_max) / fov) * axial_depth.size))
    angles_left = np.linspace(0.0, theta_min, count_left, endpoint=False)
    angles_right = np.linspace(theta_max, 2.0 * math.pi, count_right, endpoint=False)

    angles = np.concatenate((angles_left, angles_descending[::-1], angles_right))
    ranges = np.concatenate((
        np.full(count_left, depth_val_outside_fov, dtype=np.float64),
        radial_depth[::-1],
        np.full(count_right, depth_val_outside_fov, dtype=np.float64),
    ))

    center_radius = 0.5 * (r_max_ellipse + ranges)
    semi_major = np.maximum(0.5 * (r_max_ellipse - ranges) + ellipse_buffer, ellipse_buffer)
    semi_minor = np.full_like(semi_major, ellipse_semi_minor)
    rows = np.column_stack((
        center_radius * np.cos(angles),
        center_radius * np.sin(angles),
        semi_major,
        semi_minor,
        angles,
    ))
    if not np.all(np.isfinite(rows)):
        raise FloatingPointError("Generated ellipsoid set contains non-finite values")
    return rows
