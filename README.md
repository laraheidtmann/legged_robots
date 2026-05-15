follow these steps: 


pip install -r requirements.txt
 python3 -m venv venv
 source venv/bin/activate

 sudo apt update
 sudo apt install ros-rolling-xacro ros-rolling-urdf ros-rolling-pinocchio
sudo apt install -y liburdfdom-dev liburdfdom-tools liburdfdom-sensor4.0 liburdfdom-model-state4.0

echo 'export PYTHONPATH=/home/ubuntu/Documents/legged_robots/venv/lib/python3.12/site-packages:$PYTHONPATH' >> ~/.bashrc

source ~/.bashrc

sudo ln -sf /lib/aarch64-linux-gnu/liburdfdom_sensor.so.4.0 /lib/aarch64-linux-gnu/liburdfdom_sensor.so.6

sudo ln -sf /lib/aarch64-linux-gnu/liburdfdom_model.so.4.0 /lib/aarch64-linux-gnu/liburdfdom_model.so.6


# Legged Robots Workspace


This repository contains a ROS 2 Rolling workspace for legged robotics simulations using:


- ROS 2 Rolling

- PyBullet

- Pinocchio

- Xacro / URDF


---


# Prerequisites


Make sure the following are installed:


- ROS 2 Rolling

- Python 3.12

- `pip`

- `venv`


---


# Setup


## 1. Install Python Dependencies


From the workspace root:


```bash

pip install -r requirements.txt

```


---


## 2. Create and Activate a Virtual Environment


Create the virtual environment:


```bash

python3 -m venv venv

```


Activate it:


```bash

source venv/bin/activate

```


---


## 3. Install ROS 2 Dependencies


Update apt repositories:


```bash

sudo apt update

```


Install required ROS packages:


```bash

sudo apt install -y \

    ros-rolling-xacro \

    ros-rolling-urdf \

    ros-rolling-pinocchio

```


---


## 4. Install `urdfdom` Dependencies


Install the required `urdfdom` packages:


```bash

sudo apt install -y \

    liburdfdom-dev \

    liburdfdom-tools \

    liburdfdom-sensor4.0 \

    liburdfdom-model-state4.0

```


---


## 5. Add the Virtual Environment to `PYTHONPATH`


Add the virtual environment site-packages directory to your Python path:


```bash

echo 'export PYTHONPATH=/home/ubuntu/Documents/legged_robots/venv/lib/python3.12/site-packages:$PYTHONPATH' >> ~/.bashrc

```


Reload your shell configuration:


```bash

source ~/.bashrc

```


---


## 6. Fix `urdfdom` Shared Library Compatibility


ROS Rolling Pinocchio expects `liburdfdom` version `.so.6`, while some systems only provide `.so.4.0`.


Create compatibility symlinks:


```bash

sudo ln -sf /lib/aarch64-linux-gnu/liburdfdom_sensor.so.4.0 \

    /lib/aarch64-linux-gnu/liburdfdom_sensor.so.6


sudo ln -sf /lib/aarch64-linux-gnu/liburdfdom_model.so.4.0 \

    /lib/aarch64-linux-gnu/liburdfdom_model.so.6


sudo ln -sf /lib/aarch64-linux-gnu/liburdfdom_world.so.4.0 \

    /lib/aarch64-linux-gnu/liburdfdom_world.so.6


sudo ln -sf /lib/aarch64-linux-gnu/liburdfdom_model_state.so.4.0 \

    /lib/aarch64-linux-gnu/liburdfdom_model_state.so.6

```


---


## 7. Verify the Installation


Test Pinocchio:


```bash

python3 -c "import pinocchio as pin; print(pin.__version__)"

```


Test PyBullet:


```bash

python3 -c "import pybullet as pb; print(pb.__version__)"

```


---


## 8. Build the Workspace


From the workspace root:


```bash

colcon build --symlink-install

```


Source the workspace:


```bash

source install/setup.bash

```


---


# Running the Simulation


Example:


```bash

ros2 run pybullet_sims t2_temp

```


---


# Notes


- The `urdfdom` symlink workaround is required because of a binary compatibility mismatch between the ROS Rolling Pinocchio binaries and the system-provided `urdfdom` libraries.

- This setup was tested on ARM64 (`aarch64`) systems.

- The virtual environment is intentionally excluded from colcon builds using `COLCON_IGNORE`.

sudo ln -sf /lib/aarch64-linux-gnu/liburdfdom_world.so.4.0 /lib/aarch64-linux-gnu/liburdfdom_world.so.6

sudo ln -sf /lib/aarch64-linux-gnu/liburdfdom_model_state.so.4.0 /lib/aarch64-linux-gnu/liburdfdom_model_state.so.6