# Bicycle Gym: Controller Results and Milestone 6 (Free Exploration)

|                |                                                                       |
| -------------- | --------------------------------------------------------------------- |
| **Student**    | Mohamed Ahmed Fouad                                                   |
| **Student ID** | `24P0049`                                                             |
| **Simulation** | ROS 2 Humble, `bicycle_sim` / `bicycle_control` / `track_environment` |

## Table of Contents

1. [Controller Results](#1-controller-results)
2. [Milestone 6 Overview](#2-milestone-6-overview)
3. [Four-Wheel Ackermann Kinematics and `ros2_control`](#3-four-wheel-ackermann-kinematics-and-ros2_control)
4. [2D Kinematic vs 3D Simulation](#4-2d-kinematic-vs-3d-simulation)
5. [Deterministic MPC vs Sampling-Based MPPI](#5-deterministic-mpc-vs-sampling-based-mppi)
6. [Synthesis and Link to This Project](#6-synthesis-and-link-to-this-project)
7. [How to Reproduce the Results](#7-how-to-reproduce-the-results)
8. [References](#8-references)

---

## 1. Controller Results

All results were obtained using `lap_analyzer.py` on the default track, `centerline_0.csv`, with a perimeter of approximately 528.2 m.

The autonomous controllers use the same velocity profiler and longitudinal cruise-control system so that the main comparison is between their lateral path-tracking behavior.

### 1.1 Benchmark Leaderboard

| Controller                 | Best Lap Time (s) | Top Speed (m/s) | Mean CTE (m) | Max CTE (m) | RMS CTE (m) | Status           |
| -------------------------- | ----------------: | --------------: | -----------: | ----------: | ----------: | ---------------- |
| **Manual Teleoperation**   |             69.59 |            7.26 |        1.539 |      10.873 |       2.300 | 3 laps completed |
| **Lateral PID**            |             81.10 |            7.62 |        0.399 |       3.082 |       0.566 | 3 laps completed |
| **Pure Pursuit**           |             70.70 |            7.47 |        0.085 |       0.442 |       0.123 | 3 laps completed |
| **Extended Kinematic MPC** |         **70.21** |            6.84 |    **0.083** |   **0.418** |   **0.114** | 4 laps completed |

The run-level quantities were calculated as follows:

| Metric        | Aggregation                         |
| ------------- | ----------------------------------- |
| Best Lap Time | Minimum lap time                    |
| Top Speed     | Maximum per-lap maximum speed       |
| Mean CTE      | Mean of the per-lap mean CTE values |
| Max CTE       | Maximum per-lap maximum CTE         |
| RMS CTE       | `sqrt(mean(RMS_i²))`                |

### 1.2 Extended Kinematic MPC

|     Lap | Lap Time (s) | Max Speed (m/s) | Mean Speed (m/s) | Mean CTE (m) | Max CTE (m) | RMS CTE (m) | Mean Heading Error |
| ------: | -----------: | --------------: | ---------------: | -----------: | ----------: | ----------: | -----------------: |
|       1 |       71.264 |           6.805 |            6.299 |       0.0822 |      0.4180 |      0.1140 |              0.432 |
|       2 |       70.206 |           6.812 |            6.386 |       0.0825 |      0.4173 |      0.1146 |              0.441 |
|       3 |       70.299 |           6.789 |            6.378 |       0.0856 |      0.4174 |      0.1163 |              0.423 |
|       4 |       70.302 |           6.836 |            6.376 |       0.0823 |      0.3739 |      0.1126 |              0.409 |
| **Run** |   **70.206** |       **6.836** |        **6.360** |   **0.0832** |  **0.4180** |  **0.1144** |          **0.426** |

### 1.3 Pure Pursuit

|     Lap | Lap Time (s) | Max Speed (m/s) | Mean Speed (m/s) | Mean CTE (m) | Max CTE (m) | RMS CTE (m) | Mean Heading Error |
| ------: | -----------: | --------------: | ---------------: | -----------: | ----------: | ----------: | -----------------: |
|       1 |       71.569 |           7.474 |            6.239 |       0.0845 |      0.4417 |      0.1244 |              0.451 |
|       2 |       70.699 |           7.248 |            6.314 |       0.0847 |      0.4403 |      0.1227 |              0.444 |
|       3 |       70.722 |           7.225 |            6.312 |       0.0854 |      0.4266 |      0.1232 |              0.395 |
| **Run** |   **70.699** |       **7.474** |        **6.288** |   **0.0849** |  **0.4417** |  **0.1234** |          **0.430** |

### 1.4 Lateral PID

|     Lap | Lap Time (s) | Max Speed (m/s) | Mean Speed (m/s) | Mean CTE (m) | Max CTE (m) | RMS CTE (m) | Mean Heading Error |
| ------: | -----------: | --------------: | ---------------: | -----------: | ----------: | ----------: | -----------------: |
|       1 |       82.200 |           7.618 |            5.896 |       0.3810 |      2.2360 |      0.5365 |              0.615 |
|       2 |       81.100 |           7.507 |            5.964 |       0.3698 |      1.8766 |      0.5046 |              0.582 |
|       3 |       81.572 |           7.486 |            6.025 |       0.4456 |      3.0821 |      0.6457 |              0.607 |
| **Run** |   **81.100** |       **7.618** |        **5.962** |   **0.3988** |  **3.0821** |  **0.5655** |          **0.601** |

### 1.5 Manual Teleoperation

|     Lap | Lap Time (s) | Max Speed (m/s) | Mean Speed (m/s) | Mean CTE (m) | Max CTE (m) | RMS CTE (m) | Mean Heading Error |
| ------: | -----------: | --------------: | ---------------: | -----------: | ----------: | ----------: | -----------------: |
|       1 |       69.586 |           7.098 |            6.152 |       1.4343 |      6.6283 |      1.8151 |              0.557 |
|       2 |       72.914 |           7.259 |            5.907 |       1.2040 |      3.5728 |      1.4924 |              0.488 |
|       3 |       74.999 |           7.200 |            5.783 |       1.9786 |     10.8733 |      3.2158 |              0.543 |
| **Run** |   **69.586** |       **7.259** |        **5.947** |   **1.5390** | **10.8733** |  **2.2995** |          **0.529** |

### 1.6 Comparison Against MPC

| Controller                 | Best Lap vs MPC | RMS CTE vs MPC | Max CTE vs MPC | Lap-Time Spread (s) |
| -------------------------- | --------------: | -------------: | -------------: | ------------------: |
| **Extended Kinematic MPC** |        Baseline |       Baseline |       Baseline |                1.06 |
| **Pure Pursuit**           |         +0.49 s |     ≈8% higher |       +0.024 m |                0.87 |
| **Lateral PID**            |        +10.89 s |   ≈4.9× higher |   ≈7.4× higher |                1.10 |
| **Manual Teleoperation**   |         −0.62 s |    ≈20× higher |    ≈26× higher |                5.41 |

### 1.7 Qualitative Comparison

| Criterion          | Lateral PID               | Pure Pursuit              | MPC                                        |
| ------------------ | ------------------------- | ------------------------- | ------------------------------------------ |
| Path preview       | No                        | Look-ahead point          | Full prediction horizon                    |
| RMS CTE            | 0.566 m                   | 0.123 m                   | **0.114 m**                                |
| Maximum CTE        | 3.08 m                    | 0.44 m                    | **0.42 m**                                 |
| Best lap time      | 81.10 s                   | 70.70 s                   | **70.21 s**                                |
| Top speed          | 7.62 m/s                  | 7.47 m/s                  | 6.84 m/s                                   |
| Consistency        | Lowest                    | High                      | **High**                                   |
| Computational cost | Lowest                    | Low                       | Highest                                    |
| Actuator limits    | Clamped after calculation | Clamped after calculation | Explicitly constrained during optimization |

### 1.8 Interpretation of the Results

**MPC and Pure Pursuit are very close.** Both autonomous preview-based controllers maintained the vehicle within approximately 0.44 m of the centerline. MPC achieved slightly lower RMS and maximum CTE and was approximately 0.5 s faster over the best lap.

**Lateral PID performed substantially worse.** Its RMS CTE was approximately five times that of MPC, and its maximum CTE reached 3.08 m. Although it reached the highest instantaneous speed, its lower mean speed resulted in a lap time more than 10 seconds slower than MPC.

**MPC achieved the fastest autonomous lap despite having the lowest top speed.** Its lower peak speed was offset by more consistent speed and path tracking through the corners.

**Manual teleoperation produced the fastest individual lap but was not competitive in path accuracy.** Its best lap was 69.59 s, but the vehicle deviated by as much as 10.87 m from the centerline. The large variation between manual laps also shows substantially lower consistency.

**The first autonomous lap was slower for both MPC and Pure Pursuit.** MPC's first lap was 71.26 s compared with approximately 70.2 s on subsequent laps, while Pure Pursuit's first lap was 71.57 s compared with approximately 70.7 s later. This is consistent with the initial acceleration from the starting condition.

---

## 2. Milestone 6 Overview

Milestone 6 connects the 2D planar kinematic bicycle model used in this project to tools and approaches used in real autonomous-vehicle development.

The simulated vehicle uses the following parameters, which are shared by the simulator, controllers, and URDF/Xacro models:

| Parameter       |  Value |
| --------------- | -----: |
| Wheelbase `L`   | 1.25 m |
| Track width `W` | 1.18 m |
| Wheel radius    |  0.5 m |
| Wheel width     |  0.3 m |

Three topics were explored:

| # | Topic                                              | Question it answers                                                                   |
| - | -------------------------------------------------- | ------------------------------------------------------------------------------------- |
| 1 | Four-wheel Ackermann kinematics and `ros2_control` | What does the bicycle model hide about real steering geometry?                        |
| 2 | 2D kinematic vs 3D simulation                      | What does a kinematic simulator leave out?                                            |
| 3 | Deterministic MPC vs sampling-based MPPI           | How does a sampling-based controller differ from the MPC implemented in this project? |

---

## 3. Four-Wheel Ackermann Kinematics

### 3.1 From Bicycle to Four Wheels

The bicycle model collapses the two front wheels into one equivalent wheel located on the vehicle centerline. This abstraction assumes that both front wheels can be represented by a single steering angle `δ`, allowing the vehicle to be modeled using only its wheelbase and the steering angle.

The turning radius of the rear-axle center is

$$
R = \frac{L}{\tan\delta}
$$

This relationship is the basis of the kinematic bicycle model used by the controllers in this project. It is useful because it captures the dominant relationship between steering and vehicle curvature without requiring the individual geometry of all four wheels.

A real four-wheel vehicle, however, cannot generally use the same steering angle for both front wheels while maintaining pure rolling. During a turn, every wheel follows a different circular path around the **instantaneous center of rotation (ICR)**. The ICR is the point about which the vehicle is instantaneously rotating. For ideal no-slip motion, the velocity direction of every wheel must be tangent to its own circular path around this point.

This geometric constraint is what produces **Ackermann steering geometry**. The two front steering axes are arranged so that their projected axes intersect approximately at the ICR. Consequently, the inner front wheel follows a smaller-radius arc than the outer front wheel and must therefore rotate through a larger steering angle.

For a vehicle with wheelbase `L`, track width `W`, and rear-axle turning radius `R`, the ideal steering angles are

$$
\tan\delta_{inner} = \frac{L}{R-\frac{W}{2}}
$$

$$
\tan\delta_{outer} = \frac{L}{R+\frac{W}{2}}
$$

Since

$$
R-\frac{W}{2} < R+\frac{W}{2}
$$

the inner steering angle is always larger than the outer steering angle for a normal turn.

The difference is especially important in tight turns. If both front wheels were forced to use the same angle, their rolling directions would not point exactly toward the same ICR. At least one tire would therefore have to develop lateral slip relative to the ground. In a real vehicle this appears as **tire scrub**, which causes additional friction, tire wear, and energy loss.

Ackermann geometry therefore represents an important piece of physical vehicle behavior that is intentionally hidden by the bicycle model.

### 3.2 Instantaneous Center of Rotation

The ICR provides a useful way of visualizing the difference between the bicycle and four-wheel models.

For a bicycle model, the single equivalent front wheel and the rear axle can be treated as defining one turning circle. The vehicle is therefore characterized by one steering angle and one curvature:

$$
\kappa = \frac{\tan\delta}{L}
$$

For a four-wheel Ackermann vehicle, the left and right front wheels are located at different lateral positions. Their distances from the ICR are therefore different. The inner wheel travels along the smaller circle while the outer wheel travels along the larger circle.

The rear wheels also follow different radii. The inner rear wheel travels on a smaller-radius path than the outer rear wheel. However, because the rear wheels are not steered, the vehicle geometry must position them so that their rolling directions remain tangent to their respective paths.

This illustrates why track width becomes relevant once the individual wheels are modeled. The bicycle model effectively assumes that the two front wheels can be collapsed onto the centerline without changing the vehicle's overall path. This is a good approximation when the vehicle is relatively narrow compared with its turning radius, but the approximation becomes less accurate as the turn becomes tighter.

### 3.3 Bicycle Model as an Approximation

The bicycle model is not necessarily an incorrect model; it is a deliberate abstraction.

Its main assumption is that the left and right wheels on each axle can be represented by an equivalent wheel located at the axle center. This removes the need to explicitly calculate:

* inner and outer steering angles;
* individual wheel trajectories;
* individual wheel velocities;
* track-width-dependent turning geometry;
* tire scrub caused by steering mismatch.

For path-tracking algorithms, this simplification is extremely useful. The controller can reason about one steering command rather than a complete four-wheel steering system.

The approximation becomes increasingly reasonable when:

* the vehicle is relatively narrow compared with its turning radius;
* steering angles are small;
* tire slip is small;
* the objective is center-of-mass or centerline path tracking rather than individual tire behavior.

It becomes less representative when:

* the vehicle makes tight turns;
* the track width is large relative to the turning radius;
* tire forces and slip become important;
* individual wheel actuation must be controlled;
* accurate wheel odometry or actuator simulation is required.

This distinction is important for this project because the controllers are being evaluated primarily as **path-tracking algorithms**, rather than as low-level steering actuators for a physical four-wheel vehicle.

### 3.4 Worked Example with This Vehicle

For this vehicle, `L = 1.25 m` and `W = 1.18 m`.

| Turn radius `R` (m) | Bicycle `δ` (deg) | Inner `δ_in` (deg) | Outer `δ_out` (deg) | Difference (deg) |
| ------------------: | ----------------: | -----------------: | ------------------: | ---------------: |
|                   5 |             14.04 |              15.83 |               12.61 |             3.22 |
|                  10 |              7.13 |               7.55 |                6.71 |             0.84 |
|                  20 |              3.58 |               3.69 |                3.47 |             0.22 |

The results show how the bicycle approximation becomes more accurate as the turning radius increases. At `R = 20 m`, the inner and outer steering angles differ by only about `0.22°`. At `R = 5 m`, the difference grows to approximately `3.22°`.

This is a useful indication of the scale of the approximation being made. On gentle sections of the track, treating the front axle as a single equivalent steering wheel introduces relatively little geometric error. In a tight corner, however, the individual wheel angles can differ noticeably.

The difference can also be understood geometrically. As `R` becomes much larger than the track width `W`, the terms `W/2` become small relative to `R`:

$$
R-\frac{W}{2} \approx R+\frac{W}{2} \approx R
$$

so both steering angles approach the bicycle-model steering angle. As the turn becomes tighter and `R` decreases, the same track width represents a larger fraction of the turning radius, increasing the difference between the inner and outer wheel angles.

### 3.5 Practical Ackermann Steering

In an actual vehicle, the steering system does not normally command the two front-wheel angles independently from the high-level path-tracking controller. Instead, the steering mechanism implements an Ackermann relationship mechanically or through an actuator controller.

A higher-level controller can therefore continue to produce a desired vehicle steering angle or curvature, while a lower-level steering system converts that command into the appropriate individual wheel angles.

Conceptually, the command chain becomes:

```text
Path-tracking controller
        ↓
Desired vehicle curvature / steering
        ↓
Ackermann steering conversion
        ↓
Inner + outer front steering angles
        ↓
Steering actuators
```

This separation is useful because it allows the path-tracking algorithm to operate at the vehicle level while the actuator layer handles the physical geometry.

The same principle applies to longitudinal actuation. A vehicle-level acceleration or throttle request can ultimately be converted into individual wheel torques or traction commands.

### 3.6 Takeaways

| Aspect             | Bicycle model                            | Four-wheel Ackermann                                 |
| ------------------ | ---------------------------------------- | ---------------------------------------------------- |
| Steering input     | One angle `δ`                            | Inner and outer steering angles                      |
| Geometry           | Wheelbase `L`                            | Wheelbase `L` and track width `W`                    |
| Tire scrub         | Not modeled                              | Minimized by Ackermann geometry                      |
| Turning center     | Represented through one equivalent wheel | Defined by individual wheel geometry and the ICR     |
| Wheel trajectories | One equivalent path per axle             | Different radius for every wheel                     |
| Controller output  | `/steer`, `/throttle`                    | Individual wheel/actuator commands                   |
| Best use           | Path-tracking algorithm development      | Realistic vehicle actuation and hardware integration |

---

## 4. 2D Kinematic vs 3D Simulation

The simulator used in this project is primarily a **kinematic** simulator. It assumes that the vehicle follows the commanded kinematic motion without explicitly modeling tire slip and complex vehicle dynamics.

This makes it fast and useful for developing and benchmarking path-tracking algorithms.

A 3D physics simulator such as Gazebo can model additional physical effects.

| Effect                            | 2D Kinematic Simulation                | 3D Physics Simulation                       |
| --------------------------------- | -------------------------------------- | ------------------------------------------- |
| Tire slip and friction saturation | Ignored                                | Can be modeled                              |
| Suspension                        | Not modeled                            | Can be modeled                              |
| Body roll/pitch                   | Not modeled                            | Can be modeled                              |
| Load transfer                     | Not modeled                            | Can be modeled                              |
| Sensor noise                      | Not inherent                           | Can be simulated through sensor plugins     |
| Sensor latency                    | Not inherent                           | Can be simulated                            |
| Terrain/contact                   | Simple planar model                    | Physical contact and terrain                |
| Computational cost                | Very low                               | Higher                                      |
| Repeatability                     | High                                   | Depends on physics solver and configuration |
| Typical purpose                   | Algorithm development and benchmarking | System validation and realism               |

**Gazebo (Gz-Sim)** provides a general-purpose physics simulation environment with a large ecosystem of sensors, plugins, models, and ROS integrations.

**MVSim** provides a lighter-weight alternative aimed at mobile-robot and vehicle simulation. It can provide more physical effects than a purely kinematic model while generally remaining lighter than a full physics environment.

### 4.1 Consequence for the Controllers

The Pure Pursuit and kinematic MPC controllers implemented in this project assume the vehicle follows a kinematic bicycle model.

At higher speeds, a real vehicle can experience lateral tire-force saturation. When this happens, the actual vehicle may understeer and follow a different trajectory from the one predicted by the kinematic model.

A dynamic-bicycle MPC incorporating tire forces and slip angles would therefore be a natural extension for a more realistic simulator.

The curvature-based velocity profiler implemented in Milestone 5 helps reduce this limitation by reducing the target velocity in high-curvature sections of the track.

---

## 5. Deterministic MPC vs Sampling-Based MPPI

### 5.1 Model Predictive Path Integral Control

Model Predictive Path Integral (MPPI) control is a sampling-based model-predictive control method.

At each control step, the controller:

1. Generates many candidate control sequences by perturbing a nominal control sequence.
2. Simulates the resulting trajectories.
3. Calculates a cost for each trajectory.
4. Assigns greater weight to lower-cost trajectories.
5. Combines the sampled controls to produce the next control action.

The weighted control update can be expressed conceptually as

$$
u^* = \sum_k w_k u_k
$$

where the weights depend exponentially on the trajectory cost:

$$
w_k \propto \exp\left(-\frac{S_k}{\lambda}\right)
$$

Here, `S_k` is the cost of sample `k` and `λ` controls the weighting temperature.

Unlike gradient-based optimization, MPPI does not require the cost function to be differentiable.

### 5.2 Comparison with the MPC Used in This Project

The MPC implemented in this project uses a kinematic bicycle model and an SLSQP numerical optimizer to optimize a finite sequence of steering and acceleration commands.

| Property               | Kinematic MPC in This Project           | Nav2 MPPI                                                  |
| ---------------------- | --------------------------------------- | ---------------------------------------------------------- |
| Optimization           | Numerical constrained optimization      | Sampling and weighted averaging                            |
| Model                  | Kinematic bicycle                       | Supports multiple motion models                            |
| Cost function          | Designed as a smooth weighted objective | Composed from configurable critics                         |
| Constraints            | Explicit actuator bounds                | Primarily handled through sampling, constraints, and costs |
| Non-smooth costs       | Less convenient                         | Naturally supported                                        |
| Obstacles              | Require explicit cost/model integration | Naturally represented through critics                      |
| Local minima           | Can affect optimization                 | Sampling provides exploration                              |
| Computation            | One structured optimization problem     | Many trajectory rollouts                                   |
| Output                 | Direct optimized control sequence       | Weighted control update                                    |
| Main tuning parameters | Horizon, weights, bounds                | Horizon, samples, noise, temperature, critic weights       |

The MPC in this project is **model-optimal within its chosen model and numerical optimization formulation**, rather than globally guaranteed to be optimal for the real vehicle.

MPPI trades some of this structured optimization for greater flexibility in handling complex and non-smooth cost functions.

---

## 6. Synthesis and Link to This Project

### 6.1 Kinematics vs Multi-Body Vehicle Models

The bicycle model is an effective abstraction for developing path-tracking controllers because a single steering angle captures the main turning geometry.

A four-wheel Ackermann model adds track width and separate inner and outer steering angles, providing a more realistic representation of the physical vehicle.

`ros2_control` provides the hardware abstraction needed to connect higher-level vehicle commands to actual actuators.

### 6.2 2D vs 3D Simulation

The 2D kinematic simulator used in this project is fast, deterministic, and easy to reproduce. This makes it appropriate for controller development and benchmarking.

It does not reproduce effects such as tire slip, suspension motion, load transfer, or realistic sensor behavior. A 3D simulator such as Gazebo or a more vehicle-focused simulator such as MVSim can provide these effects at a higher computational and modeling cost.

### 6.3 Deterministic vs Sampling-Based Control

The implemented MPC explicitly optimizes steering and acceleration over a finite horizon while respecting actuator limits.

MPPI instead explores many possible control sequences through sampling. This makes it attractive when the cost function contains obstacles, discontinuities, or other difficult-to-differentiate terms.

The two approaches therefore represent different trade-offs between structured optimization, computational cost, and flexibility.

---

## 7. How to Reproduce the Results

The ROS 2 Humble and workspace environments are sourced automatically through `~/.bashrc`.

From the workspace:

```bash
cd /path/to/workspace
colcon build --symlink-install
```

### Lateral PID

```bash
ros2 launch bicycle_sim bicycle_sim.launch.py controller:=lateral_pid
```

### Pure Pursuit

```bash
ros2 launch bicycle_sim bicycle_sim.launch.py controller:=pure_pursuit
```

### MPC

```bash
ros2 launch bicycle_sim bicycle_sim.launch.py controller:=mpc
```

### Manual Teleoperation

Start the simulator:

```bash
ros2 launch bicycle_sim bicycle_sim.launch.py controller:=teleop use_cruise_control:=true
```

Then start the keyboard teleoperation node:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

The keyboard node publishes `Twist` commands on `/cmd_vel`. The teleoperation bridge converts these commands into the vehicle's `/throttle` and `/steer` commands.

### Telemetry

The lap analyzer publishes:

```text
/telemetry/cte
/telemetry/speed
/telemetry/heading_err_deg
/telemetry/lap_time
```

For `rqt_plot`, use the message fields:

```bash
rqt_plot /telemetry/cte/data /telemetry/speed/data
```

The same topics can be inspected individually with:

```bash
ros2 topic echo /telemetry/speed --once
```

### Benchmark Procedure

For each controller:

1. Launch the corresponding controller mode.
2. Allow the vehicle to complete several full laps.
3. Record the per-lap values printed by `lap_analyzer.py`.
4. Calculate the run-level metrics using the aggregation rules in Section 1.1.
5. Repeat with the same track and controller configuration for a fair comparison.

---

## 8. References

1. Rajamani, R. *Vehicle Dynamics and Control*. Springer.
2. Coulter, R. C. *Implementation of the Pure Pursuit Path Tracking Algorithm*. Carnegie Mellon University.
3. Williams, G., Aldrich, A., Theodorou, E. A. *Model Predictive Path Integral Control: From Theory to Parallel Computation*. Journal of Guidance, Control, and Dynamics.
4. ROS 2 Documentation — `ros2_control`.
5. ROS 2 Navigation2 Documentation — MPPI Controller.
6. Gazebo / Gz-Sim Documentation.
7. MVSim Documentation.
