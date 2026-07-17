#!/bin/bash
set -eo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")"

colcon build --symlink-install --packages-select sender
source install/setup.bash
ros2 launch sender sender.launch.py "$@"
