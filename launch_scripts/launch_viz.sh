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

echo "Launcing Visualization module on [${AUTOWARE_LAUNCH_VIZ_CFG}] for [${AUTOWARE_LAUNCH_SCENARIO_CFG}]!!!"
if [ ${AUTOWARE_LAUNCH_VIZ_CFG} == "X86" ]; then
  source ${AUTOWARE_LAUNCH_SCRIPT_X86_PATH_CFG}/ros_env.sh ${AUTOWARE_LAUNCH_SCRIPT_X86_PATH_CFG}
  source ${AUTOWARE_INSTALL_X86_PATH_CFG}/setup.bash
  ros2 launch autoware_launch viz.launch.xml vehicle_model:=$VEHICLE_MODEL \
  sensor_model:=sample_sensor_kit launch_joy:=true launch_viz:=true \
  launch_topic_monitor:=false

elif [ ${AUTOWARE_LAUNCH_VEHICLE_CFG} == "TGT_FROM_X86" ]; then
  printf "%s" "Skipping VIZ & launching JoyStick controller!!!!"
  echo "source ${AUTOWARE_LAUNCH_SCRIPT_TARGET_PATH_CFG}/ros_platform_env.sh ${AUTOWARE_INSTALL_TARGET_PATH_CFG}; \
  source ${AUTOWARE_INSTALL_TARGET_PATH_CFG}/setup.bash; \
  ros2 launch autoware_launch viz.launch.xml vehicle_model:=$VEHICLE_MODEL \
  sensor_model:=sample_sensor_kit launch_joy:=true launch_viz:=false \
  launch_topic_monitor:=false" | ssh root@$TARGET_IP_CFG

elif [ ${AUTOWARE_LAUNCH_VEHICLE_CFG} == "TGT" ]; then
  printf "%s" "Skipping VIZ & launching JoyStick controller!!!!"
  source ${AUTOWARE_LAUNCH_SCRIPT_TARGET_PATH_CFG}/ros_platform_env.sh ${AUTOWARE_LAUNCH_SCRIPT_TARGET_PATH_CFG}
  source ${AUTOWARE_INSTALL_TARGET_PATH_CFG}/setup.bash
  ros2 launch autoware_launch viz.launch.xml vehicle_model:=$VEHICLE_MODEL \
  sensor_model:=sample_sensor_kit launch_joy:=true launch_viz:=false \
  launch_topic_monitor:=false

fi

bash
