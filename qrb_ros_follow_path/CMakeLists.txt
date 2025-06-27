cmake_minimum_required(VERSION 3.8)
project(qrb_ros_follow_path VERSION 1.0.2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rcutils REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(qrb_ros_amr_msgs REQUIRED)
find_package(qrb_ros_navigation_msgs REQUIRED)
find_package(qrb_ros_robot_base_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(qrb_follow_path_manager REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
ament_auto_find_build_dependencies()

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/include/topic
    ${PROJECT_SOURCE_DIR}/include/service
    ${PROJECT_SOURCE_DIR}/include/action
    ${PROJECT_SOURCE_DIR}/include/manager
    ${EIGEN3_INCLUDE_DIRS}
    ${CMAKE_SYSROOT}/usr/include/qrb_follow_path_manager/
    ${CMAKE_SYSROOT}/usr/include/qrb_follow_path_manager/include/
    ${PROJECT_SOURCE_DIR}/../install/qrb_follow_path_manager/include/qrb_follow_path_manager/
    ${PROJECT_SOURCE_DIR}/../install/qrb_follow_path_manager/include/qrb_follow_path_manager/include/
    ${PROJECT_SOURCE_DIR}/../install/include/qrb_follow_path_manager/
    ${PROJECT_SOURCE_DIR}/../install/include/qrb_follow_path_manager/include/
    )
#include_directories(${JSON_INC_PATH})
add_definitions(${EIGEN3_DEFINITIONS})

set(source
  src/main.cpp
  src/navigation_controller.cpp
  src/topic/exception_subscriber.cpp
  src/topic/tf_subscriber.cpp
  src/topic/odom_subscriber.cpp
  src/topic/path_publisher.cpp
  src/topic/twist_publisher.cpp
  src/topic/laser_scan_subscriber.cpp
  src/topic/map_subscriber.cpp
  src/topic/developer_mode_subscriber.cpp
  src/service/virtual_path_service_server.cpp
  src/service/navigation_path_service_server.cpp
  src/action/follow_path_action_server.cpp
)

aux_source_directory(./src SRC_LIST)

ament_auto_add_executable(qrb_ros_follow_path ${SRC_LIST} ${source})

ament_target_dependencies(qrb_ros_follow_path
  "nav_msgs"
  "geometry_msgs"
  "rclcpp"
  "rclcpp_action"
  "std_msgs"
  "rcutils"
  "tf2"
  "nav_2d_msgs"
  "tf2_ros"
  "tf2_geometry_msgs"
  "qrb_ros_amr_msgs"
  "qrb_ros_navigation_msgs"
  "qrb_ros_robot_base_msgs"
  "sensor_msgs"
  "rclcpp_lifecycle"
  "qrb_follow_path_manager"
)

target_link_libraries(qrb_ros_follow_path
  tinyxml2
)

target_link_libraries(qrb_ros_follow_path
  "qrb_follow_path_manager"
)

set_target_properties(${PROJECT_NAME} PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION 1
)

install(TARGETS qrb_ros_follow_path
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_auto_package(INSTALL_TO_SHARE launch)
