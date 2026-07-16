#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODE="${1:-}"

cd "${ROOT_DIR}"

set -u

export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"

build_package() {
  colcon build --packages-select "$1"
  set +u
  source "${ROOT_DIR}/install/setup.bash"
  set -u
}

case "${MODE}" in
  sender)
    build_package sender
    ros2 launch sender sender.launch.py
    ;;
  receiver)
    build_package receiver
    ros2 launch receiver receiver.launch.py
    ;;
  *)
    echo "usage: $0 [sender|receiver]" >&2
    exit 2
    ;;
esac
