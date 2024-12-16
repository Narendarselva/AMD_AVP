cmake_minimum_required(VERSION 3.14)
project(avp_planner)

find_package(autoware_cmake REQUIRED)
autoware_package()

if(PARKLOT_SIM)
	add_definitions("-DPARKLOT_SIM")
else()
	set(CMAKE_CXX_FLAGS "-std=c++11 -fPIC -Wno-error")
	add_definitions("-UPARKLOT_SIM")
endif()

ament_auto_add_executable(${PROJECT_NAME}
  src/avp_planner_node.cpp
  src/avp_planner.cpp
)

ament_target_dependencies(${PROJECT_NAME})

install(
	TARGETS ${PROJECT_NAME}
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
	)
ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
