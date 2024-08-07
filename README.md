
# Unveiling cell hitRatio in slam_toolbox through a ROS2 message

**Authors**: Pascal Sikorski, Xiao Tan  
**Date**: 8/6/24  

## Overview

The ROS2 library `slam_toolbox` found here can provide a reliable and consistent SLAM method of providing a dynamic map of a robot's environment represented through an occupancy grid. Cells in the occupancy grid comprise of three states: occupied, free, and unknown.

For regular use, this is okay; however, when wishing to work within probabilities of our environment provided by SLAM, this is undesirable as instead of working in static states, we would want to work with the probability of cells: i.e., 80% likelihood of occupancy, or 20% likelihood of occupancy. To get this, we wish to extract the `hitRatio` of a cell calculated in `UpdateCell()`.

Each occupancy grid that makes up the SLAM map environment consists of a variety of cells that dynamically grow in population as the map expands through exploration. In each cell, we utilize `cellPassCnt` and `cellHitCnt` to calculate the probability that each cell is occupied through `hitRatio`, the variable we wish to extract.

In the `UpdateCell()` function of `Karto.h`, we find where this estimation occurs, first finding if there have been enough LiDAR beams interacting with the cell to make a strong enough estimate (where the default cell value would likely be unknown if this check fails), where if passing, an if-else occurs to check the threshold for `hitRatio` to determine if a cell is occupied or not. Here, `pCell` stores this updated state of the cell, and later, the function `toNavMap()` calls the occupancy grid to find specific cell state information from `GetValue()` of each cell. In this call, you can see the static states of `GridStates_Unknown`, `GridStates_Occupied`, and `GridStates_Free` being pushed to the map message.

## Goals

The final goal of this change would be able to read the cell occupancy value in a presentable manner communicated to other nodes in ROS2. In this solution, we wish to view the occupancy map with the difference being cells that can output their exact probabilistic occupancy in a message, aligned in form to what is already communicated through `/map`. We no longer can only work with the state of cells, but instead, we wish to see the exact probability of occupancy of the cell communicated in this or a similar map message.

In our briefly explored attempts at implementing this change, we attempted to publish the `hitRatio` variable alongside the rounded static state value from `pCell` in `toNavMap()`. This would require having to make a new message type from a new ROS2 node, following a similar format used for the occupancy map message [shown here](https://docs.ros.org/en/melodic/api/nav_msgs/html/msg/OccupancyGrid.html). This documentation shows that the `nav_msg` states occupancy probability is between \[0,100\] – though this is not the case in practice with `slam_toolbox` due to the traditional static cell states. It could suffice successfully implementing the change of the `pCell` value itself to be a rounded integer of probability between \[0-100\], though in doing this we must be certain that any other internal functions relying on `pCell` utilizing a state is corrected to account for this export change.

In this testing, we discovered that one of these functions, `IsFree()` requires a cell state check for `GridStates_Free`, and it is required to do ray casts within our SLAM. We need to consider this and other internal functions that currently require reading cells through their occupancy states instead of the exact probability to operate correctly, and what changes would be needed to publish probabilistic occupancy without breaking existing functions.

Otherwise, in extracting the `hitRatio`, there likely needs to be developed a messenger node capable of providing the data type to store the (ideally) double-type probability of each cell and integrate it into `slam_toolbox` parallel to the `int8[] map` state of `map.data` use. This means when we call cell 50 at index 50 of data at `data[50]`, we can assume our occupancy type indexing to be the same, where pulling an example of `occupancy[50]` with an occupancy of only 5% must interact with the same cell as `data[50]` which could show the state as free in our map. To implement this correctly, we need to ensure that the grid type stores the probability of occupancy as the entire occupancy map grows, and dynamically changes alongside the stored cell state grid during robot exploration without the potential for indexing issues.

## Environment

Our research bench uses ROS2 Humble on Ubuntu Linux - Jammy Jellyfish (22.04), an install guide can be found [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html). After a successful ROS2 install, we look at installing these packages, all from similar humble dev branches:

- [Rivz2](https://github.com/ros2/rviz/tree/humble)
- [Turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3/tree/humble-devel)
- [Turtlebot3_msgs](https://github.com/ROBOTIS-GIT/turtlebot3_msgs/tree/humble-devel)
- [Turtlebot3_simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/humble-devel)
- [Gazebo_ros_pkgs](https://github.com/ros-simulation/gazebo_ros_pkgs/tree/ros2)

These packages can be built as binaries (or as sources if wished), but we require `slam_toolbox` to be built from source in order to make and run new code changes. As well as this, we should install and build `slam_toolbox` from a new `ros2_ws` workspace directory, which details on how to make and install can be shown [here](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html). If working with a new ROS2 node to store a new message for occupancy, it’s likely best to develop that node in our new `ros2_ws` workplace as well.

- [Slam_toolbox](https://github.com/SteveMacenski/slam_toolbox/tree/humble)

To test and confirm that we installed it correctly, we can make new terminals for each node and test our `slam_toolbox` with its output by executing these commands:

**Terminal 1:**

```bash
$ source /opt/ros/humble/setup.bash
$ export TURTLEBOT3_MODEL=burger
$ ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

**Terminal 2:**

```bash
$ source /opt/ros/humble/setup.bash
$ ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True
```

**Terminal 3:**

```bash
$ source ~/ros2_ws/install/setup.bash
$ ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
```

**Terminal 4:**

```bash
$ source /opt/ros/humble/setup.bash
$ ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
```

**Terminal 5:**

```bash
$ source /opt/ros/humble/setup.bash
$ export TURTLEBOT3_MODEL=burger
$ ros2 run turtlebot3_teleop teleop_keyboard
```

To exactly replicate our environment from a new install of Ubuntu Linux - Jammy Jellyfish (22.04), follow these steps:

### Prerequisites

1. **Install Ubuntu 22.04 (Jammy Jellyfish).**
2. **Run initial update:**

```bash
$ sudo apt update
```

### Install ROS2 Humble

Follow the [official ROS2 Humble installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html). Here's our step by step:

1. **Set up locales:**
```bash
$ sudo apt update && sudo apt install locales
```
Here, we confirmed that our locale was already set as `en_US.UTF-8`, though the guide can be followed to update.

2. **Install required packages:**
```bash
$ sudo apt install software-properties-common
$ sudo add-apt-repository universe
$ sudo apt update && sudo apt install curl -y
```

3. **Add ROS2 apt repository:**
```bash
$ sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
$ echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

4. **Install ROS2 with dev tools**
```bash
$ sudo apt update
$ sudo apt upgrade
$ sudo apt install ros-humble-desktop
$ sudo apt install ros-humble-ros-base
$ sudo apt install ros-dev-tools
```

For simplicity, after a successful install, at this point we can close our current terminal and test in new ones.

5. **Test ROS2 install**

Open a new terminal, and enter this for our talker:
```bash
$ source /opt/ros/humble/setup.bash
$ ros2 run demo_nodes_cpp talker
```

Open another new terminal, with the one we just created still running, and create a listener:

```bash
$ source /opt/ros/humble/setup.bash
$ ros2 run demo_nodes_py listener
```

We should now see messages communicated back and forth with one another. Now, we `CTRL-C` kill both current running ROS2 programs, and close one terminal to still have one, sourced terminal running.

If we ever need to source a terminal to its original ROS2 install, we can use 
```bash
$ source /opt/ros/humble/setup.bash
```

6. **Install packages for slam_toolbox testing**

In our single, sourced, and ready terminal, install our necessary packages:

```bash
$ sudo apt update

$ sudo apt install ros-humble-rviz2

$ sudo apt install ros-humble-turtlebot3
$ sudo apt install ros-humble-turtlebot3-msgs
$ sudo apt install ros-humble-turtlebot3-simulations

$ sudo apt install ros-humble-gazebo-ros-pkgs
```

After a successful install, let's consider dependencies:

```bash
$ rosdep update
```

However, on the first run, we should be prompted that rosdep has not been initiated. If so, run:

```bash
$ sudo rosdep init
```

Then finally, run:

```bash
$ rosdep update
```

7. **Create ROS2 Workspace and Install slam_toolbox**

Similar to earlier, we are now following the [official ROS2 Humble Workspace installation guide](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html). Here's our step by step:

If we are still in the same terminal from our earlier step, continue using it. Otherwise, create a new terminal and source it using 

```bash
$ source /opt/ros/humble/setup.bash
```

Then we begin workspace creation:

```bash
$ mkdir -p ~/ros2_ws/src
$ cd ~/ros2_ws/src
```

We can test cloning as followed in the guide, and there is nothing wrong in doing so, though I will assume in following this guide that your workspace is properly configured and we will move to the `slam_toolbox` step. Assuming we are still in the still in the `src` directory, run:

```bash
$ git clone https://github.com/SteveMacenski/slam_toolbox -b humble
$ cd ..
```

Now we should be in the `ros2_ws` directory, outside of `src`

```bash
$ rosdep install -i --from-path src --rosdistro humble -y
$ colcon build --packages-select slam_toolbox
```

8. **Test run slam_toolbox with required packages**

As stated above, we now should be able to test our installation using the following commands in new terminals:

**Terminal 1:**

```bash
$ source /opt/ros/humble/setup.bash
$ export TURTLEBOT3_MODEL=burger
$ ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

(If having an issue with the robot not appearing, try to `CTRL-C` `gazebo` and try again)

**Terminal 2:**

```bash
$ source /opt/ros/humble/setup.bash
$ ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True
```

**Terminal 3:**

```bash
$ source ~/ros2_ws/install/setup.bash
$ ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
```

**Terminal 4:**

```bash
$ source /opt/ros/humble/setup.bash
$ ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
```

**Terminal 5:**

```bash
$ source /opt/ros/humble/setup.bash
$ export TURTLEBOT3_MODEL=burger
$ ros2 run turtlebot3_teleop teleop_keyboard
```

Following the prompt in `teleop_keyboard`, we should now notice the `burger` turtlebot3 model move around the simulation in `gazebo`, and in `rviz2`. The `rviz2` example should be presenting the SLAM map, as well as robot localization, as we are getting updates from the map. This map is what we can reference in our changes.

## Solutions

For successful implementation, it's required to understand how information is stored within a cell type, how these cells are stored/shown in an occupancy map, as well as how information from the occupancy map can be pushed into a ROS2 message. Other important functions to note for development include `UpdateCell()`, `GetValue()`, and `toNavMap()` in `slam_toolbox`.

For testing results, a simple ROS2 node can be made to pull data from topics that `slam_toolbox` already utilizes, such as `/map` to store our partial map data, as well as pulling the occupancy information from the desired form decided in development, either in a new message or as an existing message provided in `nav_msgs`. By comparing occupancy alongside state estimation in this new node, we can track to confirm if a cell's state estimate looks to correlate with its occupancy value at the same index while updates for both should be published simultaneously with one another. By confirming all indexes represent the same cell, and by successfully extracting the exact occupancy value data in a message (updated where our occupancy map typically would), we would then consider this change complete.

An example node has been pushed in this GitHub repo, with steps to make a new node/package shown as the following in a new terminal:

```bash
$ source /opt/ros/humble/setup.bash
$ cd ~/ros2_ws/src
$ ros2 pkg create --build-type ament_python map_listener
```

After successfully making the package, navigate to `setup.py` within the package (now in the `src` folder) and add the changes reflected in our GitHub repo upload. These changes are found in `setup.py`, which includes a new entry point formation, and `package.xml`, which includes new depends.

Then within the nested `map_listener` directory (which should have an `__init__.py` in the same folder), create `map_listener.py` and place our reflected GitHub sample node code in as well. Then build:

```bash
$ cd ~/ros2_ws
$ colcon build --packages-select map_listener --symlink-install
```

`symlink-install` allows us to make Python script changes without rebuilding, but for good practice, rebuild the package (likely the same way) after making any other package changes. Rebuilding has to occur after any changes made with `slam_toolbox` even if only script changes have been made, as there are internally compiled dependencies. To correctly view the output of the occupancy grid in our sample starter code, we can run (while `slam_toolbox` is correctly running), in a new terminal:

```bash
$ source ~/ros2_ws/install/setup.bash
$ ros2 run map_listener map_listener_node
```

## Summary

In short, we wish to extract the `hitRatio` value from each cell in the occupancy grid in `slam_toolbox` through a message in ROS2 for other nodes to understand and view from the indexing of other map data. We desire this, as for our work, viewing cell probability as “a cell existing with 80% certainty” is much more useful than a state estimation of “this cell exists/doesn't exist”. With this data extraction, being able to index the exact occupancy of cells in a map alongside traditional map data allows for a seamless integration into our existing work, and would be necessary to formulate a map for our use.
