# Simulation Setup

The following section explains how to get the Gazebo simulation up and running.

## Clone Simulation Repo from GitHub

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

If Gazebo is not yet installed, run:

```
sudo apt-get install ros-humble-ros-gz
rosdep install --from-path src -yi
```

Then try building the package again.

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

# Adding a Camera

Navigate to the directory `src/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_burger/`. Open 2 model files: `model.sdf` and `model-1_4.sdf`. In **both** files, locate the section that begins with `<link name="base_scan">`, then add the following code just above the closing `</link>` tag:

```xml
<sensor type="camera" name="camera">
    <update_rate>10</update_rate>
    <pose>-0.032 0 0.171 0 0 0</pose>
    <visualize>true</visualize>
    <camera name="head">
        <!--Approximate field-of-view of pi cam-->
        <horizontal_fov>1.089</horizontal_fov>
        <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
        </image>
        <clip>
        <near>0.05</near>
        <far>8.0</far>
        </clip>
        <noise>
        <type>gaussian</type>
        <!-- Noise is sampled independently per pixel on each frame.
                That pixel's noise value is added to each of its color
                channels, which at that point lie in the range [0,1]. -->
        <mean>0.0</mean>
        <stddev>0.007</stddev>
        </noise>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <ros>
        <!-- TODO(louise) Remapping not working due to https://github.com/ros-perception/image_common/issues/93 -->
        <argument>image_raw:=image_demo</argument>
        <argument>camera_info:=camera_info_demo</argument>
        </ros>
        <!-- camera_name>omit so it defaults to sensor name</camera_name-->
        <!-- frame_name>omit so it defaults to link name</frameName-->
    </plugin>
    </sensor>
```

This will create a virtual camera within the Gazebo simulation that publishes to the `image_raw` topic.
