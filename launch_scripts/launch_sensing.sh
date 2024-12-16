#!/bin/bash
echo "Sourcing the config file [${AUTOWARE_LAUNCH_COFIG_FILE_ENV}]"
. ${AUTOWARE_LAUNCH_COFIG_FILE_ENV}

echo "Launcing Sensing module on [${AUTOWARE_LAUNCH_VEHICLE_CFG}]!!!"

if [ ${AUTOWARE_LAUNCH_PLATFORM_CFG} != "SIM" ]; then
   VEHICLE_MODEL="lexus"
   IS_SIM="false"
else
   VEHICLE_MODEL="sample_vehicle"
   IS_SIM="true"
fi

if [ ${AUTOWARE_LAUNCH_PLATFORM_CFG} != "SIM" ]; then
  if [ ${AUTOWARE_LAUNCH_SENSING_CFG} == "X86" ]; then
    source ${AUTOWARE_LAUNCH_SCRIPT_X86_PATH_CFG}/ros_env.sh ${AUTOWARE_LAUNCH_SCRIPT_X86_PATH_CFG}
	  source ${AUTOWARE_INSTALL_X86_PATH_CFG}/setup.bash
	  ros2 launch autoware_launch sensing.launch.xml \
    vehicle_model:=$VEHICLE_MODEL \
    sensor_model:=sample_sensor_kit gnss_ip:=$GNSS_IP_CFG launch_uls:=true
  elif [ ${AUTOWARE_LAUNCH_SENSING_CFG} == "TGT_FROM_X86" ]; then
	  echo "
    screen -S AVP-DEMO -X screen -t "SENSING" bash -c  \
    \"source ${AUTOWARE_LAUNCH_SCRIPT_TARGET_PATH_CFG}/ros_platform_env.sh ${AUTOWARE_LAUNCH_SCRIPT_TARGET_PATH_CFG}; \
    source ${AUTOWARE_INSTALL_TARGET_PATH_CFG}/setup.bash; \
    ros2 launch autoware_launch sensing.launch.xml \
    vehicle_model:=$VEHICLE_MODEL sensor_model:=sample_sensor_kit \
    gnss_ip:=$GNSS_IP_CFG launch_uls:=false;exec bash\"" | ssh root@$TARGET_IP_CFG
    ssh -t root@$TARGET_IP_CFG "screen -x AVP-DEMO -p SENSING"
  elif [ ${AUTOWARE_LAUNCH_SENSING_CFG} == "TGT" ]; then
	  source ${AUTOWARE_LAUNCH_SCRIPT_TARGET_PATH_CFG}/ros_platform_env.sh ${AUTOWARE_LAUNCH_SCRIPT_TARGET_PATH_CFG}
    source ${AUTOWARE_INSTALL_TARGET_PATH_CFG}/setup.bash
    ros2 launch autoware_launch sensing.launch.xml \
    vehicle_model:=$VEHICLE_MODEL sensor_model:=sample_sensor_kit \
    gnss_ip:=$GNSS_IP_CFG launch_uls:=false
  else
  	printf "%s" "Skipping Sensing module as it is set to NO!!!!"
  fi
else
	printf "%s" "Skipping Sensing module as platform is not SIM!!!!"
fi

bash
