#!/bin/bash
echo "Sourcing the config file [${AUTOWARE_LAUNCH_COFIG_FILE_ENV}]"
. ${AUTOWARE_LAUNCH_COFIG_FILE_ENV}

echo "Launcing Vehicle module on [${AUTOWARE_LAUNCH_VEHICLE_CFG}] for [${AUTOWARE_LAUNCH_SCENARIO_CFG}]!!!"

if [ ${AUTOWARE_LAUNCH_PLATFORM_CFG} != "SIM" ]; then
   VEHICLE_MODEL="lexus"
   IS_SIM="false"
   VEHICLE_ID="default"
else
   VEHICLE_MODEL="sample_vehicle"
   IS_SIM="true"
   VEHICLE_ID="default"
fi

if [ ${AUTOWARE_LAUNCH_VEHICLE_CFG} == "X86" ]; then    
    source ${AUTOWARE_LAUNCH_SCRIPT_X86_PATH_CFG}/ros_env.sh $AUTOWARE_LAUNCH_SCRIPT_X86_PATH_CFG
    source ${AUTOWARE_INSTALL_X86_PATH_CFG}/setup.bash
    ros2 launch autoware_launch vehicle.launch.xml \
    vehicle_model:=$VEHICLE_MODEL sensor_model:=sample_sensor_kit \
    vehicle_id:=$VEHICLE_ID use_socketcan:=true socketcan_device:=can5 \
    vehicle_simulation:=$IS_SIM

elif [ ${AUTOWARE_LAUNCH_VEHICLE_CFG} == "TGT_FROM_X86" ]; then
    echo "
    screen -S AVP-DEMO -p 0 -X title \"VEHICLE\"; \
    screen -S AVP-DEMO -p 0 -X stuff \
    \"source ${AUTOWARE_LAUNCH_SCRIPT_TARGET_PATH_CFG}/ros_platform_env.sh $AUTOWARE_LAUNCH_SCRIPT_TARGET_PATH_CFG; \
    source ${AUTOWARE_INSTALL_TARGET_PATH_CFG}/setup.bash; \
    ros2 launch autoware_launch vehicle.launch.xml \
    vehicle_model:=$VEHICLE_MODEL \
    sensor_model:=sample_sensor_kit \
    vehicle_id:=$VEHICLE_ID vehicle_simulation:=$IS_SIM use_socketcan:=false \
    use_r5fcan:=true$(printf \\r)\"" | ssh root@$TARGET_IP_CFG
    ssh -t root@$TARGET_IP_CFG "screen -x AVP-DEMO -p 0"

elif [ ${AUTOWARE_LAUNCH_VEHICLE_CFG} == "TGT" ]; then
    source ${AUTOWARE_LAUNCH_SCRIPT_TARGET_PATH_CFG}/ros_platform_env.sh $AUTOWARE_LAUNCH_SCRIPT_TARGET_PATH_CFG
    source ${AUTOWARE_INSTALL_TARGET_PATH_CFG}/setup.bash
    ros2 launch autoware_launch vehicle.launch.xml \
    vehicle_model:=$VEHICLE_MODEL \
    sensor_model:=sample_sensor_kit \
    vehicle_id:=$VEHICLE_ID vehicle_simulation:=$IS_SIM use_socketcan:=false \
    use_r5fcan:=true

else
    printf "%s" "Skipping Vehicle module!!!!"
fi

bash
