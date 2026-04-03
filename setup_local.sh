#!/bin/bash
echo "Setup unitree ros2 simulation environment"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "[ERROR] ROS2 Humble not found: /opt/ros/humble/setup.bash"
  return 1 2>/dev/null || exit 1
fi

if [ ! -f "${SCRIPT_DIR}/cyclonedds_ws/install/setup.bash" ]; then
  echo "[ERROR] Build first: ${SCRIPT_DIR}/cyclonedds_ws"
  return 1 2>/dev/null || exit 1
fi

source /opt/ros/humble/setup.bash
source "${SCRIPT_DIR}/cyclonedds_ws/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="lo" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'

