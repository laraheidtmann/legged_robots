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
ros2 run tutorial_4 standing_01

#or

ros2 run tutorial_4 one_leg_stand_02

#or 

ros2 run tutorial_4 squatting_03


```



## Tutorial 5: Balance Control

### Scripts

#### t51.py
Implements balance control for the Talos humanoid robot using **torque control**. The
robot is simulated standing on two feet with a whole-body QP controller (TSID) that
directly commands joint torques. Two balance strategies are implemented:

- **Ankle strategy**: shifts the CoM reference in response to ZMP displacement to keep
  the ZMP inside the support polygon (eq. 11).
- **Hip strategy**: generates whole-body angular momentum in response to CMP displacement
  to counteract larger disturbances (eq. 12).

A state machine applies a sequence of external pushes (right, left, back) to test
disturbance rejection. Ground reference points (ZMP, CMP, CP, CoM) are estimated from
ankle force-torque sensors and published to ROS for visualisation.

#### t52.py
A copy of t51 that replaces torque control with a **position-controlled hardware
interface**. Instead of commanding joint torques directly, the TSID accelerations are
integrated to obtain virtual joint positions and velocities, which are then commanded to
the robot. This simulates how balance control would be deployed on hardware that only
exposes a position interface.

---

### How to run

#### t51 — Torque control with balance strategies

```bash
ros2 run tutorial_5 t51 --ros-args \
  -p force_magnitude:=40.0 \
  -p control_strategy:=both_strategies
```

Available strategies: `ankle_strategy`, `hip_strategy`, `both_strategies`

#### t52 — Position-controlled hardware interface

```bash
ros2 run tutorial_5 t52 --ros-args \
  -p force_magnitude:=40.0 \
  -p control_strategy:=both_strategies
```

#### Recording a bag

```bash
timeout 60 ros2 bag record -o my_bag /zmp /cmp /cp /com 
```

---

## Questions

### 4. Which ground reference points can exist outside the supporting polygon?

The CMP and CP/DCM can exist outside the supporting polygon.

The ZMP cannot, since it is the point where the resultant ground reaction
wrench has zero moment, which is only physically meaningful inside the contact surface.
If the ZMP were to exit the support polygon, the foot would tip and the contact model
breaks down.

The CMP can exit the polygon when the net angular momentum rate is non-zero (i.e.
the hip strategy is active or the robot is rotating). It represents where the GRF line
of action pierces the ground, which is not constrained to the support polygon.

The CP/DCM can also exist outside — it represents the point the robot would need to
step to in order to stop, which during a large disturbance can be well outside the
current support polygon (which is exactly when stepping becomes necessary).

---

### 5. Which modality holds higher pushing forces — torque or position hardware interface?

Torque control holds higher pushing forces. TSID directly commands joint torques
computed from the full whole-body QP, giving the most direct and accurate implementation
of the balance controller. The position control modality introduces an additional
integration step (accelerations --> velocities --> positions) which adds lag and reduces the
effective bandwidth of the controller, making it slower to react to sudden disturbances.

---

### 6. Are the torque and position control modalities equivalent?

No, because the position modality integrates the TSID accelerations to get virtual joint
positions and velocities, then commands those to the hardware. This is only equivalent
to torque control if the position controller on the hardware is infinitely stiff and
fast, which in practice it is not. The position interface acts like an implicit PD controller
on top of the TSID solution, which:

- Adds damping and lag not present in the torque formulation
- Means the robot tracks the TSID solution rather than executing it directly
- Makes the effective control gains depend on the hardware PD gains, not just the TSID weights

The result is a more compliant, slower response that can handle some disturbances better
(the implicit damping absorbs impacts) but loses the precise force control that makes
TSID optimal.