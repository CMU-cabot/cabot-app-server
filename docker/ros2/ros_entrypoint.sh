#!/bin/bash
set -e

# Default values
HOST_UID=${HOST_UID:-1000}
HOST_GID=${HOST_GID:-1000}
HOST_TZ=${HOST_TZ:-UTC}

if [[ $TZ != $HOST_TZ ]]; then
    # Setting up timezone
    sudo ln -snf /usr/share/zoneinfo/$HOST_TZ /etc/localtime
    echo $HOST_TZ | sudo tee /etc/timezone
    export TZ=$HOST_TZ
fi

CONT_UID=$(id -u developer)
CONT_GID=$(id -g developer)
if [[ $CONT_UID -ne $HOST_UID ]] || [[ $CONT_GID -ne $HOST_GID ]]; then
    # Update user and group ID to match host
    sudo usermod -u $HOST_UID developer
    sudo groupmod -g $HOST_GID developer
fi

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --
if [ -f /opt/custom_ws/install/setup.bash ]
then
  source /opt/custom_ws/install/setup.bash
fi

if [ -f /home/developer/ros2_ws/install/setup.bash ]
then
  source /home/developer/ros2_ws/install/setup.bash
fi

export DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/$HOST_UID/bus

exec gosu developer "$@"
