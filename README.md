# ros2_walker
A demonstration of a simple turtlebot walking algorithm which moves until it is close to an obstacle, at which it continuously rotates till the path is clear to move forward. It uses LaserScan for obtaining distances.

## Dependencies
- Ubuntu 20.04
- ROS2 Humble
- Gazebo
- TurtlBot3

## Steps to build
1. Set turtlebot3_model to waffle_pi:

`export TURTLEBOT3_MODEL=waffle_pi`
2. Source your ROS2 workspace:

`source <ros2_installation_dir>/install/setup.bash`
3. git clone repo:

`https://github.com/niteshjha08/ros2_walker`

4. Source your colcon workspace:

`source <ws>/install/setup.bash`

## Steps to run

1. Turtlebot launch:
`ros2 launch ros2_walker turtlebot3_walker.launch.py`

2. Optionally, if you want to record all topics except /camera/* :
`ros2 launch ros2_walker turtlebot3_walker.launch.py record:=1`

## Inspect bag file:
`ros2 bag info <ros_bag_name>`

## Playing back the recorded bag file:
`ros2 bag play <ros_bag_name>`