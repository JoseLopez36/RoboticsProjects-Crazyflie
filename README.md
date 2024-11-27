## MRS-Crazyflies
This ROS2 package contains adapted configuration files and launch files from [CrazySim](https://github.com/gtfactslab/CrazySim), that should be used in the 2nd part of the MRS Project for the simulation part.

Simulation part is run in physics simulator Gazebo Ignition and used ROS2 packages are based on [CrazySwarm2](https://imrclab.github.io/crazyswarm2/)

## Installation

Again, there are two ways you can set up your computer to run the simulation:
1. **Using Docker** (recommended!!!)
2. If you **already have ROS2** installed and having hard time using docker on your laptop.

### 1) Docker installation (recommended!!!)

Clone the [repository](https://github.com/larics/docker_files):
```
git clone https://github.com/larics/docker_files.git
```
And follow the setup instructions there. This docker automatically clones and builds ROS2 mrs_crazyflies package.

### 2) Manual installation (if you already have ROS2 installed)
> We are assuming that you have ROS2 Humble installed.

Please follow the instructions given on the [CrazySim](https://github.com/gtfactslab/CrazySim) page to setup simulation. Additionally check for aliases script: https://github.com/larics/docker_files/tree/ros-humble-cf/ros2/ros2-humble/crazyflies/to_copy and README in this repository which might come in handy.

The folder structure of this package is:
1. maps - it contains .bmp and .yaml files that are used in map server.
2. worlds -  here are located .sdf files and 3D models based on the 2D files in folder maps.
3. scripts - additional node for static transformation broadcaster from world to odom is there. 
4. launch -  it contains file to launch gazebo simulation with crazyflies (sitl_multiagent_text.sh) with the initial poses from file in folder drone spawn list, launch file to start map server(map_server_launch.py) and launch file which starts crazyflies server, rviz and nodes for publishing velocity to crazyflies.
5. config - here is the configuration file for rviz and the main yaml file for crazyflies server
6. startup - it contains the example of starting the simulation and ROS2 nodes.

## Topics and services

Velocity commands are published on `/cf_x/cmd_vel` to crazyflie cf_x. Odometry can be obatined from the topic `/cf_x/odom` . Pose can also be obtained from the topic `/cf_x/pose` and velocity from `/cf_x/velocity`, just mind that for this topic message type is not Twist. 
To take off/land you can call services  `/cf_x/takeoff`, `/cf_x/land`. Current vel_mux.py does takeoff automatically, after the first cmd_vel command, but you can call it on your own. 


## Test the simulation
> [!NOTE]
> Within MRS docker, the `mrs_crazyflies` package is located in `/root/CrazySim/ros2_ws/src/`. All folders and files mentioned later in these instructions are located inside the package In docker has been added alias cd_mrs_crazyflies which changes the directory to this package.

This example showcases how to run the simulation using sessions, tmuxinator and environment variables. You do not need to use this format if you do not find it useful.

To run the example, navigate to `startup` folder and run:
```
./start.sh
```
It will open one window with several panes.

#### 1. The first pane starts the gazebo simulation:
```
 bash /path-to-workspace/ros2_ws/src/mrs_crazyflies/launch/sitl_multiagent_text.sh -m crazyflie -f $SPAWN_POSE_DOC -w $ENV_NAME
```
Please notice that an example assumes that the installation is done in the docker. If you didn't use docker, you may have different path. Bash script that starts gazebo requires several arguments -m is for the model. Please always use crazyflie. -f stands for the .txt file with the x and y iniitial positions for each crazyflie. The example for 4 crazyflies is given in the folder `launch/drone_spawn_list` (feel free to add yours here) and -w requires the world name which can be found in the worlds folder.

The environment variables `$SPAWN_POSE_DOC` and `$ENV_NAME`, alongside the `$NUM_ROB`, which defines number of robots, are located in `mrs_example_setup.sh`. This file should be sourced, alongside ros2 workspaces before (alias: ros2_ws and source_ros2) - check out pre_window in `session.yml`. :)

#### 2. In the second pane (up right), ROS2 crazyflies server, rviz and crazyflie nodes that publish cmd_vel, are started.
```
 waitForCfGazebo;sleep 5; ros2 launch mrs_crazyflies cf_velmux_launch.py
```
The shell function `waitForCfGazebo` waits until crazyflies are spwaned in gazebo plus additional 5 seconds of sleep, just in case, to have enough time to start. It can be found in [to_copy](https://github.com/larics/docker_files/tree/ros-humble-cf/ros2/ros2-humble/crazyflies/to_copy) aliases (in docker it is copied to `/root/.bash_aliases`).

Crazyflies server takes the data from `crazyflies_mrs.yaml`. For more info please read about: [CrazySim](https://github.com/gtfactslab/CrazySim) and [CrazySwarm2](https://imrclab.github.io/crazyswarm2/).

**Please keep in mind that the variable `$NUM_ROB` should correspond to the number of enabled crazyflies in the `crazyflies_mrs.yaml` and the number of rows in the `$SPAWN_POSE_DOC` also, otherwise the server won't be able to connect with gazebo. Also initial positions in `$SPAWN_POSE_DOC` should correspond to the ones in `crazyflies_mrs.yaml`.** Feel free to change them according to your task.

If you are waiting in this second pane, and it doesn't say that 'All Crazyflies loggging are initialized', please check this [issue](https://github.com/gtfactslab/CrazySim/issues/1#issue-2123839637) and its [solution](https://github.com/gtfactslab/CrazySim/issues/1#issuecomment-1933212957). Just keep in mind that world (.sdf) files that you need to adapt are in this package's world directory.

#### 3. The third pane (bottom left) starts the [map server](https://github.com/ros-navigation/navigation2/tree/main/nav2_map_server)
```
waitForCfGazebo; ros2 launch mrs_crazyflies map_server_launch.py map_yaml:=/root/CrazySim/ros2_ws/src/mrs_crazyflies/maps/$ENV_NAME/$ENV_NAME.yaml
```

#### 4. The fourth pane (bottom right) is given as an example to test if crazyflie cf_1 is moving.
The command is stored in history, so you need to move in that pane, press up arrow, and press enter, when everything else is already on.

```
history -s "ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/cf_1/cmd_vel"
```
After killing the session using ctrl+b, k, there might be some ros2 nodes running in the background, please do the command: kill_ros2, which will kill all ros2 processes running, it is defined in .bash_aliases. Keep this in mind when starting next session. :)
## Working on your project

For developing your solution, you can either create a new package, or you can continue to work in this package. You can write your code in Python or C++.

Feel free to add more windows or to create your own setups and sessions. :)


