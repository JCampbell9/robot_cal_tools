cmake_minimum_required(VERSION 3.5)
project(rct_ros_tools_rclcpp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(rct_image_tools REQUIRED)
find_package(rct_optimizations REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(boost_plugin_loader REQUIRED)

find_package(ament_cmake REQUIRED)
set(ROS2_DEPS
  rclcpp
  cv_bridge
  image_transport
  tf2_ros
  tf2_eigen
  std_srvs
  image_transport
)
foreach(dep ${ROS2_DEPS})
  find_package(${dep} REQUIRED)
endforeach()

###########
## Build ##
###########

# Executable for collecting data sets via subscribers and triggered with services
# See readme (TODO: Write a readme)
add_executable(${PROJECT_NAME}_cmd src/command_line_cal.cpp)
set_target_properties(${PROJECT_NAME}_cmd PROPERTIES OUTPUT_NAME command_line_data_collection PREFIX "")
target_include_directories(${PROJECT_NAME}_cmd PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
ament_target_dependencies(${PROJECT_NAME}_cmd ${ROS2_DEPS})
target_link_libraries(${PROJECT_NAME}_cmd
 rct::rct_image_tools
 rct::rct_optimizations
 Boost::filesystem
 boost_plugin_loader::boost_plugin_loader
)

add_executable(${PROJECT_NAME}_wrist_cal_node  src/hand_eye_calibration.cpp)
set_target_properties(${PROJECT_NAME}_wrist_cal_node PROPERTIES OUTPUT_NAME hand_eye_calibration PREFIX "")
target_include_directories(${PROJECT_NAME}_wrist_cal_node PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${rclcpp_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}_wrist_cal_node ${ROS2_DEPS})
target_link_libraries(${PROJECT_NAME}_wrist_cal_node
 rct::rct_image_tools
 rct::rct_optimizations
 rct::rct_common
 boost_plugin_loader::boost_plugin_loader
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_data_loader)
ament_export_dependencies(${ROS2_DEPS})
ament_package()

#############
## Install ##
#############

set(ROS_LIB_DESTINATION lib)
set(ROS_BIN_DESTINATION bin)
set(ROS_INCLUDE_DESTINATION include)

install(TARGETS ${PROJECT_NAME}_cmd EXPORT ${PROJECT_NAME}_cmd
  ARCHIVE DESTINATION ${ROS_LIB_DESTINATION}
  LIBRARY DESTINATION ${ROS_LIB_DESTINATION}
  RUNTIME DESTINATION ${ROS_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_wrist_cal_node EXPORT ${PROJECT_NAME}_wrist_cal_node
  ARCHIVE DESTINATION ${ROS_LIB_DESTINATION}
  LIBRARY DESTINATION ${ROS_LIB_DESTINATION}
  RUNTIME DESTINATION ${ROS_BIN_DESTINATION}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

install(TARGETS ${PROJECT_NAME}_cmd
        ${PROJECT_NAME}_wrist_cal_node
        DESTINATION ${ROS_LIB_DESTINATION}/${PROJECT_NAME})
