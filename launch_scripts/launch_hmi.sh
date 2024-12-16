#!/bin/bash
echo "Sourcing the config file [${AUTOWARE_LAUNCH_COFIG_FILE_ENV}]"
. ${AUTOWARE_LAUNCH_COFIG_FILE_ENV}

if [ ${AUTOWARE_LAUNCH_PLATFORM_CFG} != "SIM" ]; then
   VEHICLE_MODEL="lexus"
   IS_SIM="false"
else
   VEHICLE_MODEL="sample_vehicle"
   IS_SIM="true"
fi

echo "Launcing HMI module on [${AUTOWARE_LAUNCH_HMI_CFG}]!!!"
if [ ${AUTOWARE_LAUNCH_HMI_CFG} == "X86" ]; then
  source ${AUTOWARE_LAUNCH_SCRIPT_X86_PATH_CFG}/ros_env.sh ${AUTOWARE_LAUNCH_SCRIPT_X86_PATH_CFG}
  source ${AUTOWARE_INSTALL_X86_PATH_CFG}/setup.bash
  cd ${AUTOWARE_INSTALL_X86_PATH_CFG}/amd_disti_hmi_interface
  ros2 launch amd_disti_hmi_interface hmi_interface.launch.xml

elif [ ${AUTOWARE_LAUNCH_HMI_CFG} == "TGT_FROM_X86" ]; then
  echo"
  screen -S AVP-DEMO -X screen -t "HMI" bash -c \
  \"source ${AUTOWARE_LAUNCH_SCRIPT_TARGET_PATH_CFG}/ros_platform_env.sh ${AUTOWARE_LAUNCH_SCRIPT_TARGET_PATH_CFG}; \
  source ${AUTOWARE_INSTALL_ZCU102_PATH_CFG}/setup.bash; \
  cd ${AUTOWARE_INSTALL_ZCU102_PATH_CFG}/amd_disti_hmi_interface; \
  ros2 launch amd_disti_hmi_interface hmi_interface.launch.xml;exec bash\"" | ssh root@$TARGET_IP_CFG
  ssh -t root@$TARGET_IP_CFG "screen -x AVP-DEMO -p HMI"
elif [ ${AUTOWARE_LAUNCH_HMI_CFG} == "TGT" ]; then
  source ${AUTOWARE_LAUNCH_SCRIPT_TARGET_PATH_CFG}/ros_platform_env.sh AUTOWARE_LAUNCH_SCRIPT_TARGET_PATH_CFG
  source ${AUTOWARE_INSTALL_ZCU102_PATH_CFG}/setup.bash
  cd ${AUTOWARE_INSTALL_ZCU102_PATH_CFG}/amd_disti_hmi_interface
  ros2 launch amd_disti_hmi_interface hmi_interface.launch.xml

else
  printf "%s" "Skipping HMI!!!!"
fi

bash
