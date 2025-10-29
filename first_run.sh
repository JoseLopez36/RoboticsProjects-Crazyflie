#!/bin/bash

# If not working, first do: sudo rm -rf /tmp/.docker.xauth
# It still not working, try running the script as root.

XAUTH=/tmp/.docker.xauth

echo "Preparing Xauthority data..."
xauth_list=$(xauth nlist :0 | tail -n 1 | sed -e 's/^..../ffff/')
if [ ! -f $XAUTH ]; then
    if [ ! -z "$xauth_list" ]; then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

echo "Done."
echo ""
echo "Verifying file contents:"
file $XAUTH
echo "--> It should say \"X11 Xauthority data\"."
echo ""
echo "Permissions:"
ls -FAlh $XAUTH
echo ""
echo "Running docker..."

# Require the host path to this repository's root to be provided via env var
if [ -z "$ROBOTICSPROJECTS_CRAZYFLIE_REPO_ROOT" ]; then
    echo "ERROR: Please set ROBOTICSPROJECTS_CRAZYFLIE_REPO_ROOT to the absolute path of this repo on your host (e.g., /home/youruser/.../roboticsprojects_crazyflie)."
    exit 1
fi

# Hook to the current SSH_AUTH_LOCK - since it changes
# https://www.talkingquickly.co.uk/2021/01/tmux-ssh-agent-forwarding-vs-code/
ln -sf $SSH_AUTH_SOCK ~/.ssh/ssh_auth_sock

docker run -it \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="TERM=xterm-256color" \
    --device=/dev/ttyUSB0 \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="/dev:/dev" \
    --volume="/var/run/dbus/:/var/run/dbus/:z" \
    --volume="$ROBOTICSPROJECTS_CRAZYFLIE_REPO_ROOT/roboticsprojects_crazyflie:/root/ros2_ws/src/roboticsprojects_crazyflie:rw" \
    --volume ~/.ssh/ssh_auth_sock:/ssh-agent \
    --env SSH_AUTH_SOCK=/ssh-agent \
    --net=host \
    --privileged \
    --name roboticsprojects_crazyflie_cont \
    roboticsprojects_crazyflie_img
