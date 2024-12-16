#!/bin/bash
echo "Sourcing the config file [${AUTOWARE_LAUNCH_COFIG_FILE_ENV}]"
. ${AUTOWARE_LAUNCH_COFIG_FILE_ENV}

echo "Launcing Perception module on [${AUTOWARE_LAUNCH_PERCEPTION_CFG}]!!!"

if [ ${AUTOWARE_LAUNCH_PLATFORM_CFG} != "SIM" ]; then
   VEHICLE_MODEL="lexus"
   IS_SIM="false"
else
   VEHICLE_MODEL="sample_vehicle"
   IS_SIM="true"
fi

if [ ${AUTOWARE_LAUNCH_PERCEPTION_CFG} == "X86" ]; then
  source ${AUTOWARE_LAUNCH_SCRIPT_X86_PATH_CFG}/ros_env.sh ${AUTOWARE_LAUNCH_SCRIPT_X86_PATH_CFG}
  source ${AUTOWARE_INSTALL_X86_PATH_CFG}/setup.bash
  ros2 launch autoware_launch perception.launch.xml \
    vehicle_model:=$VEHICLE_MODEL sensor_model:=sample_sensor_kit \
    launch_dummy_apa_perception:=false launch_motovis_perception:=true \
    use_simulated_parking:=$IS_SIM

elif [ ${AUTOWARE_LAUNCH_PERCEPTION_CFG} == "TGT_FROM_X86" ]; then
  echo "
  screen -S AVP-DEMO -X screen -t "PERCEPTION" bash -c  \
  \"source ${AUTOWARE_LAUNCH_SCRIPT_TARGET_PATH_CFG}/ros_platform_env.sh ${AUTOWARE_LAUNCH_SCRIPT_TARGET_PATH_CFG}; \
  export LD_LIBRARY_PATH=/mnt/lib; \
  source ${AUTOWARE_INSTALL_TARGET_PATH_CFG}/setup.bash; \
  export DISPLAY=:10.0; \
  ros2 launch autoware_launch perception.launch.xml \
    vehicle_model:=$VEHICLE_MODEL	sensor_model:=sample_sensor_kit \
    launch_dummy_apa_perception:=true launch_motovis_perception:=true \
    use_simulated_parking:=$IS_SIM;exec bash\"" | ssh root@$TARGET_IP_CFG
  ssh -t root@$TARGET_IP_CFG "screen -x AVP-DEMO -p PERCEPTION"

elif [ ${AUTOWARE_LAUNCH_PERCEPTION_CFG} == "TGT" ]; then
  source ${AUTOWARE_LAUNCH_SCRIPT_TARGET_PATH_CFG}/ros_platform_env.sh ${AUTOWARE_LAUNCH_SCRIPT_TARGET_PATH_CFG}
  export LD_LIBRARY_PATH=/mnt/lib
  source ${AUTOWARE_INSTALL_TARGET_PATH_CFG}/setup.bash
  export DISPLAY=:10.0
  ros2 launch autoware_launch perception.launch.xml \
    vehicle_model:=$VEHICLE_MODEL	sensor_model:=sample_sensor_kit \
    launch_dummy_apa_perception:=true launch_motovis_perception:=true \
    use_simulated_parking:=$IS_SIM

else
  printf "%s" "Skipping perception module!!!!"
fi

bash
