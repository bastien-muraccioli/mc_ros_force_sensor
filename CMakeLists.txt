cmake_minimum_required(VERSION 3.1)

project(
  RosForceSensor
  LANGUAGES CXX
  VERSION 1.0.0)

include(CTest)
enable_testing()

# Find mc_rtc if we are not building inside mc_rtc itself
if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc REQUIRED)
endif()

if(NOT TARGET mc_rtc::mc_rtc_ros)
  message(FATAL_ERROR "mc_rtc ROS plugin is required to build this project")
endif()

if(NOT DEFINED ENV{ROS_VERSION})
  message(FATAL_ERROR "ROS_VERSION is not set in the environment")
endif()

set(plugin_HDR)

message("=========== ROS_VERSION : $ENV{ROS_VERSION}")
if($ENV{ROS_VERSION} EQUAL 1)
  set(ROS_VERSION 1)
  find_package(catkin REQUIRED COMPONENTS roscpp)
  catkin_package(CATKIN_DEPENDS geometry_msgs roscpp roslib std_msgs)
  message("=========== Catkin include directories : ${catkin_INCLUDE_DIRS}")
  include_directories(include ${catkin_INCLUDE_DIRS} $ENV{HOME}/.local/include)
  link_directories(${catkin_LIBRARY_DIRS} $ENV{HOME}/.local/lib)

  list(APPEND plugin_HDR include/utils/ROSSubscriber.h)
elseif($ENV{ROS_VERSION} EQUAL 2)
  set(ROS_VERSION 2)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  # ament_package()

  include_directories(include)
  list(APPEND plugin_HDR "include/utils/ROS2Subscriber.h")
else()
  message(FATAL_ERROR "Unknown ROS version: $ENV{ROS_VERSION}")
endif()

# Configure and install the plugin configuration
configure_file(etc/RosForceSensor.in.yaml
               "${CMAKE_CURRENT_BINARY_DIR}/etc/RosForceSensor.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/RosForceSensor.yaml"
        DESTINATION "${MC_PLUGINS_RUNTIME_INSTALL_PREFIX}/etc")

set(plugin_SRC src/RosForceSensor.cpp)

list(APPEND "src/RosForceSensor.h")

# The add_plugin macro is provided by mc_rtc
add_plugin(RosForceSensor ${plugin_SRC} ${plugin_HDR})
target_link_libraries(RosForceSensor PUBLIC mc_rtc::mc_rtc_ros)

# if(BUILD_TESTING)
#   add_subdirectory(tests)
# endif()
