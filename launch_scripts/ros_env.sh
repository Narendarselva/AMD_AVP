#!/bin/bash
RED=`tput setaf 1`
RESET=`tput sgr0`
BOLD=`tput bold`
AUTOWARE_LAUNCH_SCRIPT_X86_PATH_CFG=$1

export OVERRIDE_LAUNCH_PROCESS_OUTPUT=screen
export ROS_LOG_DIR=/tmp/
export RCUTILS_COLORIZED_OUTPUT=1
export ROS_DOMAIN_ID=10
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

if [ -e "${AUTOWARE_LAUNCH_SCRIPT_X86_PATH_CFG}/cyclonedds_config.xml" ]; then
  export CYCLONEDDS_URI=${AUTOWARE_LAUNCH_SCRIPT_X86_PATH_CFG}/cyclonedds_config.xml
else
  echo "${RED}${BOLD}!!!!cyclonedds_config.xml not found at ${AUTOWARE_LAUNCH_SCRIPT_X86_PATH_CFG}!!!!${RESET}"
  return 0
fi

echo -e ${SYSTEM_PWD_CFG} | sudo -S sysctl net.ipv4.ipfrag_time=3
echo -e ${SYSTEM_PWD_CFG} | sudo -S sysctl net.ipv4.ipfrag_high_thresh=134217728
if [ ! -e /tmp/cycloneDDS_configured ]; then
    sudo sysctl -w net.core.rmem_max=2147483647
    sudo ip link set lo multicast on
    touch /tmp/cycloneDDS_configured
fi
echo "Done sourcing the env"
