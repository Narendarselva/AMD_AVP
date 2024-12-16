#!/bin/bash

print_usage(){
echo echo "$0 usage:" && grep " .)\ #" $0; exit 0; 
}
[ $# -eq 0 ] && print_usage

while getopts ":c:" o; do
    case "${o}" in
        c) # Specify configuration file
            AUTOWARE_LAUNCH_CONFIG_FILE=${OPTARG}
            ;;
        *) # Display help
            print_usage
	    exit 0
            ;;
    esac
done
echo "!!!Launching autoware with config file [${AUTOWARE_LAUNCH_CONFIG_FILE}]!!!"
echo "***************************************************************"
cat $AUTOWARE_LAUNCH_CONFIG_FILE
echo "***************************************************************"
echo "!!!Sourcing the configuartion file!!!"
. $AUTOWARE_LAUNCH_CONFIG_FILE

# Retrieve the CPU architecture using 'uname -m'
cpu_architecture=$(uname -m)
echo "CPU Architecture: $cpu_architecture"
if [ ${cpu_architecture} == "x86_64" ]; then
  AUTOWARE_LAUNCH_SCRIPT_PATH_CFG=$AUTOWARE_LAUNCH_SCRIPT_X86_PATH_CFG
  export AUTOWARE_LAUNCH_COFIG_FILE_ENV=$AUTOWARE_LAUNCH_SCRIPT_X86_PATH_CFG/launch.config
else
  AUTOWARE_LAUNCH_SCRIPT_PATH_CFG=$AUTOWARE_LAUNCH_SCRIPT_TARGET_PATH_CFG
  export AUTOWARE_LAUNCH_COFIG_FILE_ENV=$AUTOWARE_LAUNCH_SCRIPT_TARGET_PATH_CFG/launch.config
fi

if [[ "$AUTOWARE_LAUNCH_VEHICLE_CFG" == "TGT_FROM_X86" ||
  "$AUTOWARE_LAUNCH_PLANNING_CFG" == "TGT_FROM_X86" ||
  "$AUTOWARE_LAUNCH_CONTROL_CFG" == "TGT_FROM_X86" ||
  "$AUTOWARE_LAUNCH_PERCEPTION_CFG" == "TGT_FROM_X86" ||
  "$AUTOWARE_LAUNCH_LOCALIZATION_CFG" == "TGT_FROM_X86" ||
  "$AUTOWARE_LAUNCH_SENSING_CFG" == "TGT_FROM_X86" ||
  "$AUTOWARE_LAUNCH_SYSTEM_CFG" == "TGT_FROM_X86" ||
  "$AUTOWARE_LAUNCH_API_CFG" == "TGT_FROM_X86" ||
  "$AUTOWARE_LAUNCH_VIZ_CFG" == "TGT_FROM_X86" ||
  "$AUTOWARE_LAUNCH_HMI_CFG" == "TGT_FROM_X86" ||
  "$AUTOWARE_LAUNCH_HTOP_CFG" == "TGT_FROM_X86" ||
  "$AUTOWARE_LAUNCH_MAP_CFG" == "TGT_FROM_X86" ||
  "$AUTOWARE_LAUNCH_SIM_CFG" == "TGT_FROM_X86"  ]];then
   printf "%s" "waiting for Target ..."
   while ! ping -c 1 -n -w 1 $TARGET_IP_CFG &> /dev/null
   do
     printf "%c" "."
   done
   printf "\n%s\n"  "TARGET is back online"
   printf "\n%s\n"  "Time sync Target using NTP"
   echo "ntpdate -u $HOST_IP_CFG" | ssh root@$TARGET_IP_CFG
   echo "screen -SX AVP-DEMO quit;sleep 0.5;screen -dmS AVP-DEMO -c $AUTOWARE_LAUNCH_SCRIPT_TARGET_PATH_CFG/screenrc" | ssh root@$TARGET_IP_CFG
fi

command1="$AUTOWARE_LAUNCH_SCRIPT_PATH_CFG/launch_vehicle.sh"
command2="$AUTOWARE_LAUNCH_SCRIPT_PATH_CFG/launch_system.sh"
command3="$AUTOWARE_LAUNCH_SCRIPT_PATH_CFG/launch_sensing.sh"
command4="$AUTOWARE_LAUNCH_SCRIPT_PATH_CFG/launch_localization.sh"
command5="$AUTOWARE_LAUNCH_SCRIPT_PATH_CFG/launch_planning.sh"
command6="$AUTOWARE_LAUNCH_SCRIPT_PATH_CFG/launch_control.sh"
command7="$AUTOWARE_LAUNCH_SCRIPT_PATH_CFG/launch_perception.sh"
command8="$AUTOWARE_LAUNCH_SCRIPT_PATH_CFG/launch_api.sh"
command9="$AUTOWARE_LAUNCH_SCRIPT_PATH_CFG/launch_viz.sh"
command10="$AUTOWARE_LAUNCH_SCRIPT_PATH_CFG/launch_htop.sh"
command11="$AUTOWARE_LAUNCH_SCRIPT_PATH_CFG/launch_hmi.sh"
command12="$AUTOWARE_LAUNCH_SCRIPT_PATH_CFG/launch_map.sh"
command13="$AUTOWARE_LAUNCH_SCRIPT_PATH_CFG/launch_sim.sh"
command14="$AUTOWARE_LAUNCH_SCRIPT_PATH_CFG/launch_kvs.sh"

#Quit screen if already created
screen -SX AVP-DEMO quit
sleep 0.5

## Launch a screen instance as demon
screen -dmS AVP-DEMO -c $AUTOWARE_LAUNCH_SCRIPT_PATH_CFG/screenrc

#Launch modules in individual screens
screen -S AVP-DEMO -p 0 -X title "VEHICLE"
screen -S AVP-DEMO -p 0 -X stuff "$command1$(printf \\r)"
screen -S AVP-DEMO -X screen -t "SYSTEM" bash -c "$command2; exec bash"
screen -S AVP-DEMO -X screen -t "SENSING" bash -c "$command3; exec bash"
screen -S AVP-DEMO -X screen -t "LOCALIZATION" bash -c "$command4; exec bash"
screen -S AVP-DEMO -X screen -t "PLANNING" bash -c "$command5; exec bash"
screen -S AVP-DEMO -X screen -t "CONTROL" bash -c "$command6; exec bash"
screen -S AVP-DEMO -X screen -t "PERCEPTION" bash -c "$command7; exec bash"
screen -S AVP-DEMO -X screen -t "API" bash -c "$command8; exec bash"
screen -S AVP-DEMO -X screen -t "VIZ" bash -c "$command9; exec bash"
screen -S AVP-DEMO -X screen -t "HTOP" bash -c "$command10; exec bash"
screen -S AVP-DEMO -X screen -t "HMI" bash -c "$command11; exec bash"
screen -S AVP-DEMO -X screen -t "MAP" bash -c "$command12; exec bash"
screen -S AVP-DEMO -X screen -t "SIMULATION" bash -c "$command13; exec bash"
screen -S AVP-DEMO -X screen -t "KVS APP" bash -c "$command14; exec bash"

#Attach to the screen
screen -x AVP-DEMO
