#!/usr/bin/env bash



set -e





WORKSPACE="/home/ubuntu/legged_robots"



VENV_PATH="$WORKSPACE/venv"



PYTHON_VERSION="python3.12"



VENV_SITE_PACKAGES="$VENV_PATH/lib/$PYTHON_VERSION/site-packages"





cd "$WORKSPACE"





echo "Creating virtual environment..."



python3 -m venv "$VENV_PATH"





echo "Activating virtual environment..."



source "$VENV_PATH/bin/activate"





echo "Installing Python dependencies..."



pip install --upgrade pip



pip install -r requirements.txt





echo "Adding COLCON_IGNORE to venv..."



touch "$VENV_PATH/COLCON_IGNORE"





echo "Installing ROS and system dependencies..."



sudo apt update



sudo apt install -y ros-rolling-xacro ros-rolling-urdf ros-rolling-pinocchio liburdfdom-dev liburdfdom-tools liburdfdom-sensor4.0 liburdfdom-model-state4.0









export PYTHONPATH="$VENV_SITE_PACKAGES:$PYTHONPATH"





echo "Creating urdfdom compatibility symlinks..."



sudo ln -sf /lib/aarch64-linux-gnu/liburdfdom_sensor.so.4.0 \
    /lib/aarch64-linux-gnu/liburdfdom_sensor.so.6





sudo ln -sf /lib/aarch64-linux-gnu/liburdfdom_model.so.4.0 \
    /lib/aarch64-linux-gnu/liburdfdom_model.so.6





sudo ln -sf /lib/aarch64-linux-gnu/liburdfdom_world.so.4.0 \
    /lib/aarch64-linux-gnu/liburdfdom_world.so.6





sudo ln -sf /lib/aarch64-linux-gnu/liburdfdom_model_state.so.4.0 \
    /lib/aarch64-linux-gnu/liburdfdom_model_state.so.6





sudo ldconfig





echo "Configuring git..."



git config --global user.email "ge78puq@mytum.de"



git config --global user.name "laraheidtmann"





echo "Sourcing ROS..."



source /opt/ros/rolling/setup.bash





echo "Building workspace..."



rm -rf build install log

colcon build 


echo "Sourcing workspace..."



source install/setup.bash


echo "Adding venv site-packages to PYTHONPATH in ~/.bashrc..."



echo 'export PYTHONPATH=/home/ubuntu/legged_robots/venv/lib/python3.12/site-packages:$PYTHONPATH' >> ~/.bashrc



source ~/.bashrc



echo "Verifying installation..."



python3 -c "import pinocchio as pin; print('Pinocchio:', pin.__version__)"



python3 -c "import pybullet as pb; print('PyBullet installed')"





echo "Setup complete."



echo "Run this before using the workspace:"



echo "source $VENV_PATH/bin/activate"



echo "source /opt/ros/rolling/setup.bash"



echo "source $WORKSPACE/install/setup.bash"

