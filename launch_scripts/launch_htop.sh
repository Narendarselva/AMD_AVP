#!/bin/bash
echo "Sourcing the config file [${AUTOWARE_LAUNCH_COFIG_FILE_ENV}]"
. ${AUTOWARE_LAUNCH_COFIG_FILE_ENV}

echo "Launcing Vehicle module on [${AUTOWARE_LAUNCH_HTOP_CFG}]!!!"
if [ ${AUTOWARE_LAUNCH_HTOP_CFG} == "X86" ]; then
  htop

elif [ ${AUTOWARE_LAUNCH_HTOP_CFG} == "TGT_FROM_X86" ]; then
  echo "
  screen -S AVP-DEMO -X screen -t "HTOP" bash -c \
  \"htop;exec bash\"" | ssh root@$TARGET_IP_CFG
  ssh -t root@$TARGET_IP_CFG "screen -x AVP-DEMO -p HTOP"

elif [ ${AUTOWARE_LAUNCH_HTOP_CFG} == "TGT" ]; then
  htop

else
  printf "%s" "Skipping Htop!!!!"
fi

bash
