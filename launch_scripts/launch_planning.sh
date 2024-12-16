#!/bin/bash
echo "Sourcing the config file [${AUTOWARE_LAUNCH_COFIG_FILE_ENV}]"
. ${AUTOWARE_LAUNCH_COFIG_FILE_ENV}

echo "Launcing Planning module on [${AUTOWARE_LAUNCH_PLANNING_CFG}] for [${AUTOWARE_LAUNCH_SCENARIO_CFG}]!!!"
if [ ${AUTOWARE_LAUNCH_PLATFORM_CFG} != "SIM" ]; then
   VEHICLE_MODEL="lexus"
   IS_SIM="false"
else
   VEHICLE_MODEL="sample_vehicle"
   IS_SIM="true"
fi

if [ ${AUTOWARE_LAUNCH_SCENARIO_CFG} == "AVP" ]; then
  AVP_SCENARIO="true"
else
  AVP_SCENARIO="false"
fi

if [ ${AUTOWARE_LAUNCH_PLANNING_CFG} == "X86" ]; then
  source ${AUTOWARE_LAUNCH_SCRIPT_X86_PATH_CFG}/ros_env.sh ${AUTOWARE_LAUNCH_SCRIPT_X86_PATH_CFG}
  source ${AUTOWARE_INSTALL_X86_PATH_CFG}/setup.bash
  ros2 launch autoware_launch planning.launch.xml \
    vehicle_model:=$VEHICLE_MODEL sensor_model:=sample_sensor_kit \
    launch_avp_scenario:=$AVP_SCENARIO launch_lane_driving:=true \
    launch_lane_driving_alone:=false

elif [ ${AUTOWARE_LAUNCH_PLANNING_CFG} == "TGT_FROM_X86" ]; then
  echo "
  screen -S AVP-DEMO -X screen -t "PLANNING" bash -c \
  \"source ${AUTOWARE_LAUNCH_SCRIPT_TARGET_PATH_CFG}/ros_platform_env.sh ${AUTOWARE_LAUNCH_SCRIPT_TARGET_PATH_CFG}; \
  source ${AUTOWARE_INSTALL_TARGET_PATH_CFG}/setup.bash; \
  sleep 20; \
  ros2 launch autoware_launch planning.launch.xml \
    vehicle_model:=$VEHICLE_MODEL sensor_model:=sample_sensor_kit \
    launch_avp_scenario:=$AVP_SCENARIO launch_lane_driving:=true \
    launch_lane_driving_alone:=false;exec bash\"" | ssh root@$TARGET_IP_CFG
  ssh -t root@$TARGET_IP_CFG "screen -x AVP-DEMO -p PLANNING"
elif [ ${AUTOWARE_LAUNCH_PLANNING_CFG} == "TGT" ]; then
  source ${AUTOWARE_LAUNCH_SCRIPT_TARGET_PATH_CFG}/ros_platform_env.sh ${AUTOWARE_LAUNCH_SCRIPT_TARGET_PATH_CFG}
  source ${AUTOWARE_INSTALL_TARGET_PATH_CFG}/setup.bash
  sleep 20
  ros2 launch autoware_launch planning.launch.xml \
    vehicle_model:=$VEHICLE_MODEL sensor_model:=sample_sensor_kit \
    launch_avp_scenario:=$AVP_SCENARIO launch_lane_driving:=true \
    launch_lane_driving_alone:=false

else
  printf "%s" "Skipping Planning Module!!!!"
fi

bash
