
# Legged Robots Tutorials (TUM) — Block 1 Deliverable



## Prerequisites



- ROS 2 (Humble or later)

- PyBullet

- Pinocchio

- `talos_description` package

- `simulator` package

- `bullet_sims` package

- `teleoperation` package



---



## Tutorial 2: Modeling and Simulation



Implements joint space PD control and home posture control for the Talos robot in PyBullet, with ROS 2 joint state publishing and RViz visualization.



### How to Run



**1. Build the workspace:**



```bash

cd ~/your_ws

colcon build

source install/setup.bash

```



**2. Run one of the scripts:**



| Script | Description |

|--------|-------------|

| `t21` | Joint space PD controller — holds zero position |

| `t22` | Home posture controller — splines to stable home pose |

| `t23` | Home posture + ROS 2 joint state publishing + RViz |



```bash

ros2 run bullet_sims t21

# or

ros2 run bullet_sims t22

# or

ros2 run bullet_sims t23

```



**3. For RViz visualization (t23 only), open a second terminal:**



```bash

source install/setup.bash

ros2 launch ros_visuals talos_rviz.launch.py

```



### What Happens



**t21:** The robot starts at zero configuration. The PD controller applies torques to resist gravity. Tune `Kp`/`Kd` so the robot holds a stiff upright posture — leg gains should be roughly 3x higher than upper body gains.



**t22:** The robot splines from its initial configuration to the stable home posture over a fixed duration. The home posture uses `[0, 0, -0.44, 0.9, -0.45, 0]` for both legs and `[0, -0.24, 0, -1, 0, 0, 0, 0]` for both arms.



**t23:** Same as t22, but also publishes `/joint_states` at 30 Hz for RViz visualization. Load the RobotModel and TF plugins in RViz to see the robot moving.



---



## Tutorial 3: Control



Implements inverse dynamics control in joint space and Cartesian space for the Talos robot, with interactive marker teleoperation via RViz.



### How to Run



**1. Build the workspace:**



```bash

cd ~/your_ws

colcon build

source install/setup.bash

```



**2. Launch RViz and the simulation:**



```bash

ros2 launch teleoperation talos_rviz_launch.py

```



This launches:

- The PyBullet simulation with the Talos robot

- RViz for visualization

- The interactive marker node for teleoperation



### What Happens



**0 – 5 seconds:** The robot executes a joint space spline, moving its arms from the zero configuration to the home position using inverse dynamics control.



**After 5 seconds:** The controller switches to Cartesian space control. The right hand holds its end-of-spline position. An interactive marker spawns at the hand's location in RViz.



**Teleoperation:** Drag the interactive marker in RViz to command the robot's right hand to a new target pose. The nullspace controller keeps the rest of the body stable.



### Tuning



Gains are set in `t3.py` in the `main()` function:



| Gain | Purpose |

|------|---------|

| `Kp_cart`, `Kd_cart` | Cartesian tracking stiffness/damping |

| `Kp_posture`, `Kd_posture` | Nullspace posture task gains |

| `Kp`, `Kd` | Joint space spline tracking gains |



A good rule of thumb for stable behavior: `Kd = 2 * sqrt(Kp)` (critically damped).




