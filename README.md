# ros2-tb3-env
Package to test out nav2 planners in turtlebot3.

## Install Dependencies
```bash
sudo apt install ros-foxy-navigation2
sudo apt install ros-foxy-nav2-bringup
sudo apt install ros-foxy-turtlebot3*
```
## Setup Environment Variables
```bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/foxy/share/turtlebot3_gazebo/models
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/opt/ros/foxy/share/turtlebot3_gazebo/worlds
```
## SLAM
```bash
ros2 launch tb3_env tb3_house_slam.launch.py
```
## Nav2 with Smac planner
```bash
ros2 launch tb3_env tb3_smac_navigation.launch.py
```
