#!/usr/bin/env bash
BLACK=`tput setaf 0`
RED=`tput setaf 1`
GREEN=`tput setaf 2`
YELLOW=`tput setaf 3`
BLUE=`tput setaf 4`
MAGENTA=`tput setaf 5`
CYAN=`tput setaf 6`
WHITE=`tput setaf 7`

BOLD=`tput bold`
RESET=`tput sgr0`

skip_setup=0
debug_flag=0
packages_skip_build_finished=0
packages_to_build=()
target=''
parallel_workers=10
cmake_parallel_level=1
enable_cmake_extended_log=0
cicd_flow=0

function setup_env()
{
  echo "${GREEN}${BOLD}!!!Build Configurations!!!${RESET}"

  if [ -f HOSTROSPATH.txt ]; then
    echo -e 'HOSTROSPATH.txt exists!!!'
    echo "${YELLOW}${BOLD}Current HOST ROS path used:${GREEN}$(<HOSTROSPATH.txt)${RESET}"
    read -n 1 -p "${YELLOW} ${BOLD} Select Yes(1): To use the above path or No(0): To set new ROS path again: ${RESET}" KEY
    if [ $KEY == 0 ];then
      echo -e ""
      read -t 1 -n 10000 discard
      read -p "${YELLOW} ${BOLD}Enter the ROS2 installation path:${RESET}" ROSPATHKEY
      echo "${ROSPATHKEY}" > HOSTROSPATH.txt
    fi
  else
    read -t 1 -n 10000 discard
    read -p "${YELLOW} ${BOLD}Enter the ROS2 installation path:${RESET}" ROSPATHKEY
    echo "${ROSPATHKEY}" > HOSTROSPATH.txt
  fi 

  ROSPATH=$(cat HOSTROSPATH.txt)
  echo "${GREEN}${BOLD}!!!ROS path used: ${ROSPATH}!!!${RESET}"
  echo "Sourcing ROS2 from ${BLUE}${BOLD}$ROSPATH/setup.bash${RESET}"
  source $ROSPATH/setup.bash

  echo "${GREEN}${BOLD}!!!Patching the compressed files to its location!!!${RESET}"
  set -x #echo on

  cd compresssed_files/parking_map/wixom_ofc
  cat pointcloud_map.pcd* > pointcloud_map.pcd.tgz
  tar -xvzf pointcloud_map.pcd.tgz -C ../../../parking_map/wixom_ofc/.
  cd -
  cd compresssed_files/parking_map/wixom_bank
  cat pointcloud_map.pcd* > pointcloud_map.pcd.tgz
  tar -xvzf pointcloud_map.pcd.tgz -C ../../../parking_map/wixom_bank/.
  cd -

  echo "${GREEN}${BOLD}!!!Cloning Autoware Repo!!!${RESET}"
  git clone https://github.com/autowarefoundation/autoware.git
  cd autoware
  echo "${GREEN}${BOLD}!!!Checkout 2023.06!!!${RESET}"
  git checkout release/2023.06
  echo "${GREEN}${BOLD}!!!Importing all the Autoware sources!!!${RESET}"
  mkdir src
  vcs import src < autoware.repos
  echo "${GREEN}${BOLD}!!!Importing additional deps for AW!!!${RESET}"
  vcs import src < ../additional_deps.repos --recursive

  echo "${GREEN}${BOLD}!!!Copying patch to autoware src!!!${RESET}"
  cp -rf ../patch_2023_06/src/* src/.


  echo "Checking ROS dependencies"
  ROSDEP_CHECK=$(rosdep check -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO --skip-keys=metal 2>&1)
  if [[ "$ROSDEP_CHECK" == *"ERROR"* ]]; then
    echo "${RED}${BOLD}!!!Rosdep Failed!!! $ROSDEP_CHECK${RESET}"
    exit
  fi
  echo "${GREEN}${BOLD}!!!Installing rosdep for AW module!!!${RESET}"
  rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO --skip-keys=metal

  cd -
}

function build_x86()
{
  if [ -e "HOSTROSPATH.txt" ];then
    ROSPATH=$(cat HOSTROSPATH.txt)
  else
    read -t 1 -n 10000 discard
    read -p "${YELLOW} ${BOLD}Enter the ROS2 installation path:${RESET}" ROSPATHKEY
    echo "${ROSPATHKEY}" > HOSTROSPATH.txt
  fi
  echo "${GREEN}${BOLD}!!!ROS path used: ${ROSPATH}!!!${RESET}"
  echo "Sourcing ROS2 from ${BLUE}${BOLD}$ROSPATH/setup.bash${RESET}"
  source $ROSPATH/setup.bash

  cd autoware
  append=''
  append1=''
  if [ "$packages_to_build" != "" ]; then
    echo "packages which are build:"
    append+="--packages-select"
    for pkg in "${packages_to_build[@]}";do
      echo "$pkg";
      append+=' '$pkg
    done
  fi

  if [ $debug_flag == 1 ]; then
    append1+='--event-handlers console_direct+ '
  fi
  
  if [ $packages_skip_build_finished == 1 ]; then
    append1+='--packages-skip-build-finished '
  fi

  colcon build --parallel-workers $parallel_workers --base-paths src \
    --build-base ./build/x86 --install-base ./install/x86/install --cmake-args \
    -DCMAKE_BUILD_PARALLEL_LEVEL=$cmake_parallel_level \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_FIND_DEBUG_MODE=$enable_cmake_extended_log \
    -DBUILD_TESTING=OFF -DPARKLOT_SIM=1 -DHMI_DESKTOP_BUILD=1 \
    $append1 $append
}

function print_usage(){
  echo 'Usage:
  ./setup_env.sh -t [-s] [-l] [-d] [-p] [-N] [-L] [-h] [-S] [-c]
  -t - Target platform x86
  -s - Skip setting the build configurations
  -d - Enable debug mode in colon build
  -p - List of ros2 packages to build
  -P - No parallel workers for colon build(def: 10)
  -L - CMAKE parallel build level(def: 1)
  -S - Skip a set of packages which have finished to build previously
  -l - Extended cmake logs
  -c - CICD flow
  -h - Help'
  exit 1
}

# Parse command line options using getopts
while getopts ":t:p:sdlch:P:L:S" opt; do
  case ${opt} in
    s )
      skip_setup=1
      ;;
    l )
      enable_cmake_extended_log=1
      ;;
    d )
      debug_flag=1
      ;;
    t )
      target=$OPTARG
      ;;
    h )
      print_usage
      ;;
    P )
      parallel_workers=$OPTARG
      ;;
    L )
      cmake_parallel_level=$OPTARG
      ;;
    S )
      packages_skip_build_finished=1
      ;;
    c )
      cicd_flow=1
      ;;
    p )
      OPTIND=$((OPTIND - 1))
      while [[ ${OPTIND} -le $# && ! ${!OPTIND} =~ ^- ]]; do
        packages_to_build+=("${!OPTIND}")
        OPTIND=$((OPTIND + 1))
      done
      ;;
    \? )
      echo "Invalid option: -$OPTARG" 1>&2
      print_usage
      ;;
    : )
      echo "Option -$OPTARG requires an argument." 1>&2
      print_usage
      ;;
  esac
done

if [ "$target" == "" ];then
  echo "${GREEN}${BOLD}!!!Specify target platform!!!${RESET}"
  print_usage
elif [ "$target" != "x86" -a "$target" != "ZCU102" -a "$target" != "VEK280" ];then
  echo "${GREEN}${BOLD}!!!Specify target platform!!!${RESET}"
  print_usage
fi

# Example output based on parsed options
echo "skip_setup: $skip_setup"
echo "debug_flag: $debug_flag"
echo "Target: $target"
echo "NoOfParrallelWorkers: $parallel_workers"
echo "CmakeParallelLevel: $cmake_parallel_level"
echo "packages to build: ${packages_to_build[@]}"

if [ $skip_setup == 0 ]; then
  echo "${GREEN}${BOLD}!!!Setting [$target] Environment!!!${RESET}"
  setup_env
else
  echo "${GREEN}${BOLD}!!!Skipping Setting [$target] Environment!!!${RESET}"
fi

echo "${GREEN}${BOLD}!!!Building Autoware for [$target] target!!!${RESET}"
export CMAKE_BUILD_PARALLEL_LEVEL=$cmake_parallel_level
if [ "$target" == "x86" ]; then
  build_x86
fi
