
# Legged Robots Tutorials (TUM) — Block 2 Deliverable

## Tutorial 4: Whole Body Control






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

| `standing_01` | Exercise 1: Robot stands and holds its desired position|

| `one_leg_stand_02` | Exercise 2: Robot moves COM over right foot and after 2s lifts its left foot 0.3 m up. |

| `squatting_03` | Exercise 3: Robot stands on one leg and squats. I.e after 4s the robots height is changed with a sinusodial function. After 8s the robot moves its right hand in a circle.|


```bash
ros2 run ros_visuals standing_01

#or

ros2 run ros_visuals one_leg_stand_02

#or 

ros2 run ros_visuals squatting_03


```








## Tutorial 5: Balance Control






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

| `t51` | Exercise 1-3: Robot is being pushed with variant force. Either hip strategy, ankle strategy, both or none can be applied to keep the robots balance and not fall over. Parmeters can be passed to declare which strategy is used: control_strategy={ankle_strategy, hip_strategy, both_strategy, no_strategy} and force_magnitude to adjust the force the robot is pushed with.|

| `t52` | Exercise 4: Position-controlled hardware interface |

| `plot.py` | Plots of all the ground reference points and the CoM are done in plot.py from recorded ros bags |



```bash
ros2 run tutorial_5 t51 --ros-args -p force_magnitude:=30.0 -p control_strategy:='both_strategies'

#or

 ros2 run tutorial_5 t52 --ros-args -p force_magnitude:=80.0 -p control_strate
 gy:='both_strategies'

```


