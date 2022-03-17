cmake_minimum_required(VERSION 2.8.3)
project(rover_base)

find_package(catkin REQUIRED COMPONENTS
  controller_manager diagnostic_updater geometry_msgs hardware_interface rover_msgs nmea_msgs
  realtime_tools roscpp roslaunch roslint rosserial_server sensor_msgs std_msgs teleop_twist_joy
  topic_tools)
find_package(Boost REQUIRED COMPONENTS thread chrono)

catkin_package()

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(rover_node
  src/rover_base.cpp
  src/rover_hardware.cpp
  src/rover_diagnostic_updater.cpp)
target_link_libraries(rover_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(rover_node rover_msgs_gencpp rosserial_msgs_gencpp)

add_executable(${PROJECT_NAME}_simple_joy_node src/simple_joy_node.cpp)
target_link_libraries(${PROJECT_NAME}_simple_joy_node ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_simple_joy_node
  PROPERTIES OUTPUT_NAME simple_joy_node PREFIX "")
add_dependencies(${PROJECT_NAME}_simple_joy_node rover_msgs_gencpp)

install(TARGETS rover_node ${PROJECT_NAME}_simple_joy_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(NOT WIN32)
  install(PROGRAMS scripts/calibrate_compass
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
else()
  install(PROGRAMS scripts/calibrate_compass.bat
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endif()

install(PROGRAMS scripts/compute_calibration
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

roslaunch_add_file_check(launch/base.launch)
roslint_cpp()
roslint_add_test()
