# Simulation

## Clone from GitHub

Go to the `src` directory of your ROS workspace (create it using `mkdir src` if it doesn't already exist)

```console
cd ~/turtlebot3_ws/src/
```

Then run the following:

```
git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
```

Go back to the root of your ROS2 workspace (in this case, the `turtlebot3_ws` directory) and build the package:

```
cd ~/turtlebot3_ws
colcon build --symlink-install --packages-select turtlebot3_fake_node turtlebot3_gazebo turtlebot3_simulations
```

## General Setup

Every time you start a new terminal, you must source your local ROS workspace and specify your turtlebot model. From your ROS workspace, run:

```console
source install/local_setup.bash
export TURTLEBOT3_MODEL=burger
```

Additionally, before you start Gazebo, you must initialize it by running:

```
source /usr/share/gazebo/setup.sh
```

## Empty World

To run the simulation in an empty world, run:

```
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

## Turtlebot3 World

To run the simulation in a world with obstacles, run:

```
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

## Turtlebot3 House

To run the simulation in a world with obstacles, run:

```
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

# Teleoperation

To control the turtlebot with your keyboard, run the following:

```
export TURTLEBOT3_MODEL=burger
ros2 run turtlebot3_teleop teleop_keyboard
```

# Collecting Map Data

To start collecting map data, run:

```
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

To save the map, open a new terminal and run:

```
export TURTLEBOT3_MODEL=burger
ros2 run nav2_map_server map_saver_cli -f ~/turtlebot3_ws/map
```
