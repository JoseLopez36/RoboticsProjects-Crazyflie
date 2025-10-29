## RoboticsProjects-Crazyflie
This ROS2 package contains a fork from [MRS-Crazyflies](https://github.com/larics/mrs_crazyflies). The main goal is to simulate and deploy control, planning and coordination algorithms to Crazyflie UAVs.

Simulation part is run in physics simulator Gazebo Ignition and used ROS2 packages are based on [CrazySwarm2](https://imrclab.github.io/crazyswarm2/)

## Installation

### Docker installation
Clone the [this repository](https://github.com/JoseLopez36/roboticsprojects_crazyflie):
```
git clone https://github.com/JoseLopez36/roboticsprojects_crazyflie
```
Add  to  `~/.bashrc` and source it, or type in the current terminal:
```
export DOCKER_BUILDKIT=1
```
Run Dockerfile from the project root directory using the following commands:
```bash
# Build the Dockerfile.
# To install ros1_bridge and ROS Noetic set the argument INSTALL_BRIDGE to true.
# Otherwise set it to false, and it will only install ROS2.
docker build -t roboticsprojects_crazyflie_img .

# Run the crazysim_img2 container for the fist time
export ROBOTICSPROJECTS_CRAZYFLIE_REPO_ROOT="/absolute/path/to/this/repo/root/on/host"  # REQUIRED
./first_run.sh

# This will create docker container crazyswarm_container and position you into the container
```

For future runs, you can use the following commands:
```bash
# Start the container:
docker start -i roboticsprojects_crazyflie_cont

# Open the container in another terminal, while it is already started:
docker exec -it roboticsprojects_crazyflie_cont bash

# Stop the conatainer
docker stop roboticsprojects_crazyflie_cont

# Delete the container
docker rm roboticsprojects_crazyflie_cont

```
The docker contains packages for crazyflies simulator [CrazySim](https://github.com/gtfactslab/CrazySim). General information about Crazyflies can be found [here](https://www.bitcraze.io/products/crazyflie-2-1/).

> [!NOTE]
> The ros2 workspace is located in /root/ros2_ws
> [!IMPORTANT]
> To live-edit code from your host (e.g., in VSCode), set the environment variable `ROBOTICSPROJECTS_CRAZYFLIE_REPO_ROOT` to the absolute path of this repository's root on your host before running `./first_run.sh`. The repo will be mounted into the container at `/root/ros2_ws/src/roboticsprojects_crazyflie`.

## Test the simulation
> [!NOTE]
> Within the provided Docker image, the `roboticsprojects_crazyflie` package is located in `/root/ros2_ws/src/`. All folders and files mentioned later in these instructions are located inside the package. In Docker, there is an alias `cd_roboticsprojects_crazyflie` which changes the directory to this package.

This example showcases how to run the simulation using sessions, tmuxinator and environment variables. You do not need to use this format if you do not find it useful.

To run the example, navigate to `startup` folder and run:
```
./start.sh
```
It will open one window with several panes.

#### 1. The first pane starts the gazebo simulation:
```
 bash /path-to-workspace/ros2_ws/src/roboticsprojects_crazyflie/launch/sitl_multiagent_text.sh -m crazyflie -f $SPAWN_POSE_DOC -w $ENV_NAME
```
Bash script that starts gazebo requires several arguments -m is for the model. Please always use crazyflie. -f stands for the .txt file with the x and y iniitial positions for each crazyflie. The example for 4 crazyflies is given in the folder `launch/drone_spawn_list` (feel free to add yours here) and -w requires the world name which can be found in the worlds folder.

The environment variables `$SPAWN_POSE_DOC` and `$ENV_NAME`, alongside the `$NUM_ROB`, which defines number of robots, are located in `single_agent_setup.sh`. This file should be sourced, alongside ros2 workspaces before (alias: ros2_ws and source_ros2) - check out pre_window in `session.yml`. :)

#### 2. In the second pane (up right), ROS2 crazyflies server, rviz and crazyflie nodes that publish cmd_vel, are started.
```
 waitForCfsGazebo;sleep 2; ros2 launch roboticsprojects_crazyflie cf_velmux_launch.py
```
The shell function `waitForCfsGazebo` waits until all crazyflies are spwaned in gazebo plus additional 5 seconds of sleep, just in case, to have enough time to start. It can be found in to_copy/ aliases (in docker it is copied to `/root/.bash_aliases`).

Crazyflies server takes the data from `single_agent_crazyflie_sim.yaml`. For more info please read about: [CrazySim](https://github.com/gtfactslab/CrazySim) and [CrazySwarm2](https://imrclab.github.io/crazyswarm2/).

**Please keep in mind that the variable `$NUM_ROB` should correspond to the number of enabled crazyflies in the `single_agent_crazyflie_sim.yaml` and the number of rows in the `$SPAWN_POSE_DOC` also, otherwise the server won't be able to connect with gazebo. Also initial positions in `$SPAWN_POSE_DOC` should correspond to the ones in `single_agent_crazyflie_sim.yaml`.** Feel free to change them according to your task.

#### 3. The third pane (bottom left) starts the [map server](https://github.com/ros-navigation/navigation2/tree/main/nav2_map_server)
```
waitForCfsGazebo; ros2 launch roboticsprojects_crazyflie map_server_launch.py map_yaml:=/root/ros2_ws/src/roboticsprojects_crazyflie/maps/$ENV_NAME/$ENV_NAME.yaml
```

#### 4. The fourth pane (bottom right) is given as an example to test if crazyflie cf_1 is moving.
The command is stored in history, so you need to move in that pane, press up arrow, and press enter, when everything else is already on.

```
history -s "ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/cf_1/cmd_vel"
```
After killing the session using ctrl+b, then press k, there might be some ros2 nodes running in the background, please do the command: kill_ros2, which will kill all ros2 processes running, it is defined in .bash_aliases.