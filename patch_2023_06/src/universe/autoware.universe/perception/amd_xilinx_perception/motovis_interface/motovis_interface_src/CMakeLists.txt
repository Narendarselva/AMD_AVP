cmake_minimum_required(VERSION 3.14)
project(motovis_interface)

find_package(autoware_cmake REQUIRED)
autoware_package()

add_definitions("-DPARKLOT_SIM")
add_definitions("-UAVP") #change to AVP for dummy parking lots around ego vehicle 

ament_auto_add_executable(${PROJECT_NAME}
  src/motovis_interface_node.cpp
  src/motovis_interface.cpp
)

target_link_libraries(${PROJECT_NAME}
     -lpthread
     -lopencv_videoio
     -lopencv_imgcodecs
     -lopencv_highgui
     -lopencv_imgproc
     -lopencv_core
    )

ament_target_dependencies(${PROJECT_NAME})

install(
	TARGETS ${PROJECT_NAME}
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
	)

#ament_export_libraries(mvslot RPASegFreespace SlotDetect)
ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
