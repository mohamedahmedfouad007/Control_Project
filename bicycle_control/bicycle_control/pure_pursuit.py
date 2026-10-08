"""
High-Level Lateral Steering Controller: Geometric Pure Pursuit.
Calculates steering curvature from lookahead arc geometry.
"""

import math  # noqa: F401
import numpy as np  # noqa: F401


class PurePursuitController:
    """Adaptive Pure Pursuit lateral controller."""

    def __init__(self, wheelbase=1.25, kv=4, l_min=100, l_max=700,
                 max_steer_rad=math.radians(35.0)):
        self.L = wheelbase
        self.kv = kv
        self.l_min = l_min
        self.l_max = l_max
        self.max_steer_rad = max_steer_rad

    def compute_lookahead(self, v):
        """Adaptive lookahead distance: Ld = clip(kv * v + l_min, l_min, l_max)."""
        # TODO: Milestone 5.3 Step 1 — Adaptive Lookahead Horizon
        # The car looks further ahead at higher speeds to plan smoother turns.
        # Implement the speed-scaled lookahead formula and clamp it to the allowed range.
        Ld = float(np.clip(self.kv * v + self.l_min, self.l_min, self.l_max))
        return Ld

    def find_target_waypoint(self, x, y, path_points, lookahead):
        """Searches along path for the target waypoint at lookahead distance."""
        # TODO: Milestone 5.3 Step 2 — Target Waypoint Selection
        # This selects the goal point the car will steer toward.
        # Find the nearest waypoint on the path, then walk forward until
        # you reach one that is at least 'lookahead' meters away.
        minimum = float('inf')
        for i, path_point in enumerate(path_points):
            dist = math.sqrt(math.pow(path_point[0] - x, 2) + math.pow(path_point[1] - y, 2))
            if dist < minimum:
                minimum = dist
                target_index = i
        for offset in range(len(path_points)):
            i = (target_index + offset) % len(path_points)
            waypoint_dist = math.sqrt(math.pow(x - path_points[i][0] , 2) + math.pow(y - path_points[i][1] , 2))
            if waypoint_dist >= lookahead:
                return i, path_points[i]
            

    def compute_steering(self, x, y, yaw, target_pt, lookahead):
        """Computes steering angle in radians using Pure Pursuit geometry."""
        # TODO: Milestone 5.3 Steps 3 & 4 — Coordinate Transformation & Arc Law
        # This is the core of Pure Pursuit: transform the target into the vehicle's
        # local frame, then use the arc geometry formula to compute the steering angle.
        x_dis = target_pt[0] - x
        y_dis = target_pt[1] - y
        x_loc = math.cos(yaw) * x_dis + math.sin(yaw) * y_dis
        y_loc = -math.sin(yaw) * x_dis + math.cos(yaw) * y_dis
        true_lookahead = math.hypot(x_loc , y_loc)
        k = 2 * y_loc / math.pow(true_lookahead, 2)
        steer_command = math.atan(self.L * k)
        steer_command = float(np.clip(steer_command, -self.max_steer_rad, self.max_steer_rad))
        return steer_command
