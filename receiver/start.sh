#!/bin/bash
set -eo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")"

colcon build --packages-select receiver

source install/setup.bash

ros2 launch receiver receiver.launch.py
