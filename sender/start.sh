#!/usr/bin/env bash
set -eo pipefail

if (( $# != 0 )); then
  echo "usage: $0" >&2
  exit 2
fi

PACKAGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${PACKAGE_DIR}"

set -u

colcon build --packages-select sender
set +u
source "${PACKAGE_DIR}/install/setup.bash"
set -u

ros2 launch sender sender.launch.py
