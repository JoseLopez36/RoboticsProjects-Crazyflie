## MRS-Crazyflies 
This ROS2 package contains adapted configuration files and launch files from [CrazySim](https://github.com/gtfactslab/CrazySim), that should be used in the 2nd part of the MRS Project for the simulation part. 

Simulation part is ran in physics simulator Gazebo Ignition and ROS2 packages are based on [CrazySwarm2](https://imrclab.github.io/crazyswarm2/)

## Installation 

Again, there are two ways you can set up your computer to run the simulation:
1. **Using Docker** (recommended!!!)
2. If you **already have ROS2** installed and having hard time using docker on your laptop.

### 1) Docker installation (recommended!!!)

Clone the [repository](https://github.com/larics/docker_files):
```
git clone https://github.com/larics/docker_files.git
```
And follow the setup instructions there. This docker atomatically clones and builds this ROS2 mrs_crazyflies package. 

### 2) Manual installation (if you already have ROS2 installed)
> We are assuming that you have ROS2 Humble installed.

Please follow the instructions given on the [CrazySim](https://github.com/gtfactslab/CrazySim) page to setup simulation. Additionally check for aliases script: https://github.com/larics/docker_files/tree/ros-humble-cf/ros2/ros2-humble/crazyflies/to_copy and README in this repository which might come in handy. 

The folder structure of this package is: 
1. maps - it contains .bmp and .yaml files that are used in map server.
2. worlds -  here are located .sdf files and 3D models based on the 2D files in folder maps
3. launch -  it contains file to launch gazebo simulation with crazyflies (sitl_mulziagent_text.sh) with the initial poses taken from file in folder drone spawn list, launch file to start map server(map_server_launch.py) and launch file which starts crazyflies server, rviz and nodes for publishing velocity for each crazyflie. 
4. config - here is the config for rviz and the main yaml file for crazyflies server
5. startup - it contains the example of starting the simulation and ROS2 nodes. 

## Test the simulation
This example showcases how to run the simulation with session, tmuxinator and environment variables. You do not need to use this format if you do not find it useful.

To run the example, navigate to `startup` folder and run:
```
./start.sh
```
There will open one window with several panes. 

The first pane starts the gazebo simulation: 
```
 bash /path-to-workspace/ros2_ws/src/mrs_crazyflies/launch/sitl_multiagent_text.sh -m crazyflie -f $SPAWN_POSE_DOC -w $ENV_NAME
```
Please notice that an example is given for the installation in the docker. If you didn't use docker, you may have different path. Bash script that starts gazebo requires several arguments -m is for the model. Please always use crazyflie. -f stands for the .txt file with the x and y positions for each crazyflie. The example for 4 crazyflies is given in the folder drone_spawn_list (feel free to add yours here) and -w requires the world name which can be found in the worlds folders

The environment variables $SPAWN_POSE_DOC and $ENV_NAME, alongside the $NUM_ROB, which defines number of robots are located in mrs_example_setup.sh. This file should be sourced, alongside ros2 worspaces before, check pre_window in session.yml. :)

In the second pane ROS2 crazyflies server, rviz and crazyflie nodes to publish cmd_vel are started.
```
 waitForCfGazebo;sleep 5; ros2 launch mrs_crazyflies cf_velmux_launch.py
```
The shell function waitForCfGazebo, waits until crazyflies are spwaned in gazebo, it can be found in https://github.com/larics/docker_files/tree/ros-humble-cf/ros2/ros2-humble/crazyflies/to_copy aliases, additional 5 seconds of sleep just in case to have enough time to start. 

Crazyflies server takes the data from crazyflies_mrs.yaml. For more info please read about: [CrazySim](https://github.com/gtfactslab/CrazySim) and [CrazySwarm2](https://imrclab.github.io/crazyswarm2/) 
**Please keep in mind that the variable $NUM_ROB should correspond to the number of enabled crazyflies in the crazyflies_mrs.yaml and the number of rows in the $SPAWN_POSE_DOC also, otherwise the server won't be able to connect with gazebo**

The third pane starts the [map server](https://github.com/ros-navigation/navigation2/tree/main/nav2_map_server) 
```
waitForCfGazebo; ros2 launch mrs_crazyflies map_server_launch.py map_yaml:=/root/CrazySim/ros2_ws/src/mrs_crazyflies/maps/$ENV_NAME/$ENV_NAME.yaml
```
The fourth pane is given as an example to test if crazyflie cf_1 is moving. It is given in history, so you need to move in that pane press upper arrow and press enter when everything is on already.

```
history -s "ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/cf_1/cmd_vel"
```

## Working on your project

For developing your solution, you can either create a new package, or you can continue to work in this package. You can write your code in Python or C++.

Feel free to add more windows or to create your own setups and sessions. :) 


