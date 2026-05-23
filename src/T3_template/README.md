

# Tutorial 3: Control — Legged Robots (TUM)



## Overview



This tutorial implements joint space and Cartesian space control for the Talos humanoid robot in PyBullet, with ROS 2 integration and RViz visualization.



## Prerequisites



- ROS 2 (Humble or later)

- PyBullet

- Pinocchio

- `talos_description` package

- `teleoperation` package



## How to Run



### 1. Build the workspace



Navigate to your ROS 2 workspace and build:



```bash

cd ~/your_ws

colcon build

```



### 2. Source the workspace



```bash

source install/setup.bash

```



### 3. Launch RViz and the simulation



```bash

ros2 launch teleoperation talos_rviz_launch.py

```



This launches:

- The PyBullet simulation with the Talos robot

- RViz for visualization

- The interactive marker node for teleoperation



## What Happens



**0 – 5 seconds:** The robot executes a joint space spline, moving its arms from the zero configuration to the home position.



**After 5 seconds:** The controller switches to Cartesian space control. The right hand holds its position. An interactive marker spawns at the hand's location in RViz.



**Teleoperation:** Drag the interactive marker in RViz to move the robot's right hand to a new target pose. The nullspace controller keeps the rest of the body stable.



## Tuning



Gains are set in `t3.py` in the `main()` function:



| Gain | Purpose |

|------|---------|

| `Kp_cart`, `Kd_cart` | Cartesian tracking stiffness/damping |

| `Kp_posture`, `Kd_posture` | Nullspace posture task gains |

| `Kp`, `Kd` | Joint space spline tracking gains |



A good rule of thumb for stable behavior: `Kd = 2 * sqrt(Kp)` (critically damped).



