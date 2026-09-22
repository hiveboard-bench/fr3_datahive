#!/usr/bin/env bash
set -e
_argv=("$@")
set --

source "/opt/ros/${ROS_DISTRO}/setup.bash"

if [ -f "${CATKIN_WS}/devel/setup.bash" ]; then
    source "${CATKIN_WS}/devel/setup.bash"
fi

set -- "${_argv[@]}"

exec "$@"
