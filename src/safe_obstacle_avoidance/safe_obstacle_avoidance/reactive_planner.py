"""Pure, testable reactive goal-seeking steering law for the demo robot.

This is a local reference generator, not a global planner. A CBF-QP node
must still filter /ref_vel before commands reach the differential drive.
"""
import math
import numpy as np


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def choose_command(depth, heading_error, *, preferred_side=0,
                   cruise_speed=0.15, turn_speed=0.85,
                   caution_distance=1.65, stop_distance=0.55,
                   min_valid=0.20, max_valid=5.0):
    """Return (linear, angular, selected_side, nearest_front_distance).

    depth is a single row of AXIAL metric depth samples ordered image-left
    to image-right. Unknown ranges are treated as obstacles at min_valid.
    selected_side is +1 for turning left and -1 for right.
    """
    image = np.asarray(depth, dtype=float).reshape(-1)
    if image.size < 9:
        return 0.0, 0.0, 0, 0.0
    image = np.nan_to_num(image, nan=min_valid, posinf=max_valid, neginf=min_valid)
    image[image <= 0] = min_valid
    image = np.clip(image, min_valid, max_valid)

    # Evaluate central 24% and two adjacent 24% image sectors.
    n = image.size
    front = image[int(n * .38):int(n * .62)]
    left = image[int(n * .14):int(n * .38)]
    right = image[int(n * .62):int(n * .86)]
    def clearance(a):
        return float(np.percentile(a, 25)) if a.size else 0.0

    front_d = clearance(front)
    left_d = clearance(left)
    right_d = clearance(right)
    side = int(preferred_side) if preferred_side in (-1, 1) else 0
    speed = max(0.0, float(cruise_speed))
    turn = max(0.0, float(turn_speed))
    heading = wrap_angle(float(heading_error))

    if front_d < caution_distance:
        # Keep the prior direction if both sectors are similarly free,
        # avoiding rapid left-right oscillation at the obstacle.
        if side == 0 or abs(left_d-right_d) > 0.5:
            side = 1 if left_d >= right_d else -1
        chosen_clearance = left_d if side == 1 else right_d
        if front_d <= stop_distance or chosen_clearance <= stop_distance:
            return 0.0, side * turn, side, front_d
        ratio = max(0.0, min(1.0, (front_d-stop_distance) /
                              (caution_distance-stop_distance)))
        return speed * ratio * 0.65, side * turn, side, front_d

    # If forward is clear, align to the goal, and travel at modest speed.
    omega = max(-turn, min(turn, 1.5 * heading))
    linear = speed * max(0.0, 1.0 - abs(heading) / 1.5)
    if abs(heading) > 0.9:
        linear = 0.0
    return linear, omega, 0, front_d
