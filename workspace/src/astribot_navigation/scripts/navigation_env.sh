#!/usr/bin/env bash
# Shared S1 navigation ROS environment, also usable by native ROS clients.
NAV_SCRIPTS=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source "$NAV_SCRIPTS/real_env.sh"
NAV_WORKSPACE=$(cd "$NAV_SCRIPTS/../../.." && pwd)
for setup in "$NAV_WORKSPACE/dual_livox_ws/install/local_setup.bash" "$NAV_WORKSPACE/offline_ws/install/local_setup.bash" "$NAV_WORKSPACE/navigation_ws/install/local_setup.bash"; do
  if [ -f "$setup" ]; then source "$setup"; fi
done
export PYTHONPATH="$NAV_SCRIPTS:$NAV_SCRIPTS/../towergo_real:$NAV_SCRIPTS/../astribot_nav_sim${PYTHONPATH:+:$PYTHONPATH}"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
for arg in "$@"; do
  if [ "$arg" = '--replay' ]; then
    export ROS_DOMAIN_ID=125 ROS_LOCALHOST_ONLY=1
    unset FASTRTPS_DEFAULT_PROFILES_FILE
  fi
done
