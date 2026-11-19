#!/usr/bin/env bash
set -e
NAV_SCRIPTS=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
for arg in "$@"; do
  case "$arg" in -h|--help) PYTHONPATH="$NAV_SCRIPTS${PYTHONPATH:+:$PYTHONPATH}" exec /usr/bin/python3 -m real_navigation.session "$@" ;; esac
done
source "$NAV_SCRIPTS/navigation_env.sh" "$@"
NAV_SDK=/home/astribot/astribot_sdk_aarch64
export PYTHONPATH="$NAV_SDK/third_party/astribot_ros_middleware_py:$NAV_SDK:$PYTHONPATH"
export LD_LIBRARY_PATH="$NAV_SDK/astribot_sdk/core/common/robotics_library_py:$NAV_SDK/astribot_sdk/core/common/whole_body_control/third_party:$NAV_SDK/third_party/drake/lib:$NAV_SDK/third_party/third_pkg:/opt/drake/lib:${LD_LIBRARY_PATH:-}"
export ROBOT_TYPE=S1 ASTRIBOT_SDK_ROOT="$NAV_SDK"
exec /usr/bin/python3 -m real_navigation.session "$@"
