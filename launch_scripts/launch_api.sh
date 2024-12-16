#!/bin/bash
echo "Sourcing the config file [${AUTOWARE_LAUNCH_COFIG_FILE_ENV}]"
. ${AUTOWARE_LAUNCH_COFIG_FILE_ENV}

echo "Launcing API module on [${AUTOWARE_LAUNCH_API_CFG}] for [${AUTOWARE_LAUNCH_SCENARIO_CFG}] Scenario!!!"
if [ ${AUTOWARE_LAUNCH_PLATFORM_CFG} != "SIM" ]; then
   VEHICLE_MODEL="lexus"
   IS_SIM="false"
else
   VEHICLE_MODEL="sample_vehicle"
   IS_SIM="true"
fi

if [ ${AUTOWARE_LAUNCH_SYSTEM_CFG} == "X86" ]; then
  source ${AUTOWARE_LAUNCH_SCRIPT_X86_PATH_CFG}/ros_env.sh ${AUTOWARE_LAUNCH_SCRIPT_X86_PATH_CFG}
  source ${AUTOWARE_INSTALL_X86_PATH_CFG}/setup.bash
  ros2 launch autoware_launch api.launch.xml vehicle_model:=$VEHICLE_MODEL \
    sensor_model:=sample_sensor_kit

elif [ ${AUTOWARE_LAUNCH_SYSTEM_CFG} == "TGT_FROM_X86" ]; then
  echo "
  screen -S AVP-DEMO -X screen -t "API" bash -c \
  \"source ${AUTOWARE_LAUNCH_SCRIPT_TARGET_PATH_CFG}/ros_platform_env.sh ${AUTOWARE_LAUNCH_SCRIPT_TARGET_PATH_CFG}; \
  export LD_LIBRARY_PATH=/mnt; \
  source ${AUTOWARE_INSTALL_TARGET_PATH_CFG}/setup.bash; \
  ros2 launch autoware_launch	api.launch.xml vehicle_model:=$VEHICLE_MODEL \
    sensor_model:=sample_sensor_kit;exec bash\"" | ssh root@$TARGET_IP_CFG
  ssh -t root@$TARGET_IP_CFG "screen -x AVP-DEMO -p API"
elif [ ${AUTOWARE_LAUNCH_SYSTEM_CFG} == "TGT" ]; then
  source ${AUTOWARE_LAUNCH_SCRIPT_TARGET_PATH_CFG}/ros_platform_env.sh ${AUTOWARE_LAUNCH_SCRIPT_TARGET_PATH_CFG}
  export LD_LIBRARY_PATH=/mnt
  source ${AUTOWARE_INSTALL_TARGET_PATH_CFG}/setup.bash
  ros2 launch autoware_launch	api.launch.xml vehicle_model:=$VEHICLE_MODEL \
    sensor_model:=sample_sensor_kit

else 
  printf "%s" "Skipping API module!!!!"
fi

bash
