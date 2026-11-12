#!/usr/bin/env bash
# Source this file from the source tree or the installed package share directory.
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo 'Use: source /home/steno/agilex_tracer_ros2_ws/src/tracer_navigation/scripts/setup.bash'
  exit 1
fi

source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash" || return $?
# Load only the intended overlays, in dependency order. Generated setup.bash
# files replay the underlay chain captured at build time, including old projects.
for _tracer_overlay in \
  "${HOME}/livox_ws/install/local_setup.bash" \
  "${HOME}/fastlio2_ros2_ws/install/local_setup.bash"; do
  if [[ -f "${_tracer_overlay}" ]]; then
    source "${_tracer_overlay}" || return $?
  fi
done
if [[ -f "${HOME}/lidarloc_ws/src/lidar_localization_ros2/scripts/setup_local_env.sh" ]]; then
  # Keep the localizer's library/include paths, but bypass its recorded underlays.
  # This temporary assignment preserves any existing overlay override in the shell.
  LIDAR_LOCALIZATION_OVERLAY="${HOME}/lidarloc_ws/install/local_setup.bash" \
    source "${HOME}/lidarloc_ws/src/lidar_localization_ros2/scripts/setup_local_env.sh" || return $?
elif [[ -f "${HOME}/lidarloc_ws/install/local_setup.bash" ]]; then
  source "${HOME}/lidarloc_ws/install/local_setup.bash" || return $?
fi
source "${HOME}/agilex_tracer_ros2_ws/install/local_setup.bash" || return $?
# Humble's Cyclone DDS defaults to only ten automatic participant indices.
# This bringup has more processes. Preserve any explicitly configured profile.
if [[ "${RMW_IMPLEMENTATION:-}" == "rmw_cyclonedds_cpp" && -z "${CYCLONEDDS_URI:-}" ]]; then
  export CYCLONEDDS_URI="file://${HOME}/agilex_tracer_ros2_ws/install/tracer_navigation/share/tracer_navigation/config/cyclonedds.xml"
fi
unset _tracer_overlay
