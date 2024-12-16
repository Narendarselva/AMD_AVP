#!/bin/bash
echo "Sourcing the config file [${AUTOWARE_LAUNCH_COFIG_FILE_ENV}]"
. ${AUTOWARE_LAUNCH_COFIG_FILE_ENV}

echo "Launcing MAP module on [${AUTOWARE_LAUNCH_MAP_CFG}] for [${AUTOWARE_LAUNCH_SCENARIO_CFG}]!!!"

if [ ${AUTOWARE_LAUNCH_PLATFORM_CFG} != "SIM" ]; then
   VEHICLE_MODEL="lexus"
   IS_SIM="false"
   VEHICLE_ID="default"
else
   VEHICLE_MODEL="sample_vehicle"
   IS_SIM="true"
   VEHICLE_ID="0"
fi

if [ ${AUTOWARE_LAUNCH_SCENARIO_CFG} == "AVP" ]; then
  AVP_SCENARIO="true"
else
  AVP_SCENARIO="false"
fi

#Launch Map only for AVP scenario
if [[ ${AUTOWARE_LAUNCH_MAP_CFG} == "X86" && ${AVP_SCENARIO} == "true" ]]; then    
  source ${AUTOWARE_LAUNCH_SCRIPT_X86_PATH_CFG}/ros_env.sh ${AUTOWARE_LAUNCH_SCRIPT_X86_PATH_CFG}
  source ${AUTOWARE_INSTALL_X86_PATH_CFG}/setup.bash
  ros2 launch autoware_launch map.launch.xml \
    vehicle_model:=$VEHICLE_MODEL sensor_model:=sample_sensor_kit \
    map_path:=${AUTOWARE_LAUNCH_MAP_PATH_CFG}

elif [[ ${AUTOWARE_LAUNCH_MAP_CFG} == "TGT_FROM_X86" && ${AVP_SCENARIO} == "true" ]]; then
  echo"
  screen -S AVP-DEMO -X screen -t "MAP" bash -c  \
  \"source ${AUTOWARE_LAUNCH_SCRIPT_TARGET_PATH_CFG}/ros_platform_env.sh ${AUTOWARE_LAUNCH_SCRIPT_TARGET_PATH_CFG}; \
  source ${AUTOWARE_INSTALL_TARGET_PATH_CFG}/setup.bash; \
  ros2 launch autoware_launch map.launch.xml \
    vehicle_model:=$VEHICLE_MODEL \
    sensor_model:=sample_sensor_kit map_path:=${AUTOWARE_LAUNCH_MAP_PATH_CFG}; \
    exec bash\"" | ssh root@$TARGET_IP_CFG
  ssh -t root@$TARGET_IP_CFG "screen -x AVP-DEMO -p MAP"
elif [[ ${AUTOWARE_LAUNCH_MAP_CFG} == "TGT" && ${AVP_SCENARIO} == "true" ]]; then
  source ${AUTOWARE_LAUNCH_SCRIPT_TARGET_PATH_CFG}/ros_platform_env.sh ${AUTOWARE_LAUNCH_SCRIPT_TARGET_PATH_CFG}
  source ${AUTOWARE_INSTALL_TARGET_PATH_CFG}/setup.bash
  ros2 launch autoware_launch map.launch.xml \
    vehicle_model:=$VEHICLE_MODEL \
    sensor_model:=sample_sensor_kit map_path:=${AUTOWARE_LAUNCH_MAP_PATH_CFG}

else
  printf "%s" "Skipping Map module!!!!"
fi

bash
