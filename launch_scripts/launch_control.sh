#!/bin/bash
echo "Sourcing the config file [${AUTOWARE_LAUNCH_COFIG_FILE_ENV}]"
. ${AUTOWARE_LAUNCH_COFIG_FILE_ENV}

echo "Launcing Control module on [${AUTOWARE_LAUNCH_CONTROL_CFG}] for [${AUTOWARE_LAUNCH_SCENARIO_CFG}]!!!"
if [ ${AUTOWARE_LAUNCH_PLATFORM_CFG} != "SIM" ]; then
   VEHICLE_MODEL="lexus"
   IS_SIM="false"
   VEHICLE_ID="default"
else
   VEHICLE_MODEL="sample_vehicle"
   IS_SIM="true"
   VEHICLE_ID="default"
fi

if [ ${AUTOWARE_LAUNCH_CONTROL_CFG} == "X86" ]; then
    source ${AUTOWARE_LAUNCH_SCRIPT_X86_PATH_CFG}/ros_env.sh ${AUTOWARE_LAUNCH_SCRIPT_X86_PATH_CFG}
    source ${AUTOWARE_INSTALL_X86_PATH_CFG}/setup.bash
    ros2 launch autoware_launch control.launch.xml \
    vehicle_model:=$VEHICLE_MODEL sensor_model:=sample_sensor_kit \
    vehicle_id:=$VEHICLE_ID

elif [ ${AUTOWARE_LAUNCH_CONTROL_CFG} == "TGT_FROM_X86" ]; then
    echo "
    screen -S AVP-DEMO -X screen -t "CONTROL" bash -c \
    \"source ${AUTOWARE_LAUNCH_SCRIPT_TARGET_PATH_CFG}/ros_platform_env.sh ${AUTOWARE_LAUNCH_SCRIPT_TARGET_PATH_CFG}; \
    source ${AUTOWARE_INSTALL_TARGET_PATH_CFG}/setup.bash; \
    sleep 20; \
    ros2 launch autoware_launch control.launch.xml \
    vehicle_model:=$VEHICLE_MODEL sensor_model:=sample_sensor_kit \
    vehicle_id:=$VEHICLE_ID;exec bash\"" | ssh root@$TARGET_IP_CFG
    ssh -t root@$TARGET_IP_CFG "screen -x AVP-DEMO -p CONTROL"
elif [ ${AUTOWARE_LAUNCH_CONTROL_CFG} == "TGT" ]; then
    source ${AUTOWARE_LAUNCH_SCRIPT_TARGET_PATH_CFG}/ros_platform_env.sh ${AUTOWARE_LAUNCH_SCRIPT_TARGET_PATH_CFG}
    source ${AUTOWARE_INSTALL_TARGET_PATH_CFG}/setup.bash
    sleep 20
    ros2 launch autoware_launch control.launch.xml \
    vehicle_model:=$VEHICLE_MODEL sensor_model:=sample_sensor_kit \
    vehicle_id:=$VEHICLE_ID

else
    printf "%s" "Skipping Control module!!!!"
fi

bash
