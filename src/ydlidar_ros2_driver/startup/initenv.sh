#!/bin/sh

set -eu

if [ "$(id -u)" -ne 0 ]; then
  echo "Run this script as root: sudo $0" >&2
  exit 1
fi

script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
install -m 0644 "$script_dir/99-ydlidar.rules" /etc/udev/rules.d/99-ydlidar.rules
udevadm control --reload-rules

echo "Installed YDLidar udev rules. Unplug and reconnect the LiDAR adapter."
