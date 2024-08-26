# Formation Workspace with ROS2

## How to use
```
colcon build
source /opt/ros/humble/setup.bash
source $PWD/install/setup.bash
tmux new -d 'MicroXRCEAgent udp4 --port 8888' # once
ros2 launch formation mc_formation_launch.py
```