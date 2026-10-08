"""
High-Level Lateral Steering Controller: Extended Kinematic Bicycle MPC.
Solves a constrained non-linear program over prediction horizon N using SciPy,
optimizing steering angle and longitudinal acceleration (mapped to throttle).
"""

import math  # noqa: F401
import numpy as np  # noqa: F401
from scipy.optimize import minimize  # noqa: F401


class KinematicBicycleMPC:
    """Nonlinear Model Predictive Control for an Extended Kinematic Bicycle Model.

    Optimizes future control sequences u = [delta_k, a_k] where steering angle delta_k
    and longitudinal acceleration a_k (mapped to throttle effort) are the control inputs,
    forward-simulating a 4-state extended kinematic bicycle model x = [x, y, theta, v]^T.
    """

    def __init__(self, wheelbase=1.25, dt=0.1, horizon=10,
                 max_steer_rad=math.radians(35.0), k_a=4.0,
                 max_accel=None, max_brake=None):
        self.L = wheelbase
        self.dt = dt
        self.N = horizon
        self.max_steer_rad = max_steer_rad
        self.k_a = float(max_accel if max_accel is not None else k_a)

        # Weights: heavily penalize lateral CTE, heading error, and steering rate
        self.w_lat = 30.0
        self.w_long = 1.0
        self.w_yaw = 10.0
        self.w_v = 1.0
        self.w_steer = 0.2
        self.w_dsteer = 6.0
        self.w_accel = 0.1

        self.last_u = np.zeros(2 * self.N)  # warm-start [delta_0, a_0, delta_1, a_1, ...]

    def solve(self, x0, ref_trajectory, current_steer=0.0):
        """Solves MPC optimization problem over horizon N.

        x0: [x, y, yaw, v]
        ref_trajectory: list of length N containing [x_ref, y_ref, yaw_ref, v_ref]
        current_steer: actual current steering angle in radians
        Returns: (steer_rad, throttle_cmd in [-1.0, 1.0])
        """
        # ======================================================================
        # TODO: Milestone 5.4 — Extended Kinematic Bicycle MPC
        #
        # 1. Horizon & Bounds Setup:
        #    - Determine effective horizon N = min(self.N, len(ref_trajectory)).
        #    - If N < 2, return (0.0, 0.0).
        #    - Construct variable bounds for the decision vector:
        #      u = [delta_0, a_0, delta_1, a_1, ..., delta_N-1, a_N-1]
        #      where delta_k in [-self.max_steer_rad, self.max_steer_rad] (steering input)
        #      and a_k in [-self.k_a, self.k_a] (longitudinal acceleration input).
        
        N = min(self.N , len(ref_trajectory))
        if N < 2: return (0.0, 0.0)
        bounds = []
        for _ in range(N):
            bounds.append((-self.max_steer_rad , self.max_steer_rad))
            bounds.append((-self.k_a , self.k_a))

        # 2. Objective Function objective(u):
        #    - Unpack state [x, y, yaw, v] from x0 and set prev_delta = current_steer.
        #    - For each horizon step k in 0 .. N-1:
        #        a. Forward simulate state using discrete Extended Kinematic Bicycle equations
        #           (where longitudinal velocity v is an explicit state variable integrated
        #           forward with acceleration input a_k)
        #        b. Project tracking error into the path-aligned Frenet frame.
        #        c. Accumulate weighted quadratic costs:
        #           lateral CTE, heading error, speed error, steering, slew rate, accel.
        #        d. Update prev_delta = delta_k.
        #    - Return total cost.
        def objective(u):
            x, y, yaw, v = x0
            prev_delta = current_steer
            cost = 0
            for s in range(N):
                steer_comm = u[2 * s]
                acc_comm = u[2 * s + 1]
                x_new = x + v * math.cos(yaw) * self.dt
                y_new = y + v * math.sin(yaw) * self.dt
                yaw_new = yaw + v * math.tan(steer_comm) / self.L * self.dt
                v_new = v + acc_comm * self.dt
                ref_x, ref_y, yaw_ref, v_ref = ref_trajectory[s]
                dx = ref_x - x_new
                dy = ref_y - y_new
                cte = -math.sin(yaw_ref) * dx + math.cos(yaw_ref) * dy
                at_err = math.cos(yaw_ref) * dx + math.sin(yaw_ref) * dy
                heading_err = math.atan2(math.sin(yaw_ref - yaw_new) , math.cos(yaw_ref - yaw_new))
                speed_err = v_ref - v_new
                step_cost = self.w_lat * cte ** 2 + self.w_long * at_err ** 2 + self.w_v * speed_err ** 2 + self.w_yaw * heading_err ** 2 + self.w_steer * steer_comm ** 2 + self.w_accel * acc_comm ** 2 + self.w_dsteer * (steer_comm - prev_delta) ** 2
                cost += step_cost
                prev_delta = steer_comm
                x, y, yaw, v = x_new, y_new, yaw_new, v_new
            return cost


        # 3. Warm-Start Initialization:
        #    - Construct u_init by shifting self.last_u forward by 1 time step.
        def u_init():
            u = self.last_u[2:]
            u = np.append(u , u[2 * N - 4])
            u = np.append(u , u[2 * N - 3])
            return u

        # 4. Numerical Optimization & Control Extraction:
        #    - Call scipy.optimize.minimize(objective, u_init, bounds=bounds,
        #                                   method='SLSQP',
        #                                   options={'maxiter': 25, 'ftol': 1e-3}).
        #    - Save optimal solution in self.last_u.
        #    - Extract first control step: delta_cmd = u*[0], accel_cmd = u*[1].
        #    - Map optimal acceleration a_0* to normalized throttle in [-1.0, 1.0]:
        #      throttle_cmd = accel_cmd / self.k_a
        #    - Return tuple: (delta_cmd, throttle_cmd).
        # ======================================================================
        result = minimize(objective, u_init(), bounds=bounds,
                method='SLSQP',
                options={'maxiter': 25, 'ftol': 1e-3})
        self.last_u = result.x
        decision_throttle = result.x[1] / self.k_a
        decision_steer = result.x[0]
        return (decision_steer, decision_throttle)
        pass
