cmake_minimum_required(VERSION 3.8)
project(qrb_follow_path_manager VERSION 1.0.2)

set(library_name qrb_follow_path_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/include/common
    ${PROJECT_SOURCE_DIR}/include/controller
    ${PROJECT_SOURCE_DIR}/include/manager
    ${PROJECT_SOURCE_DIR}/include/path_planner
    ${PROJECT_SOURCE_DIR}/include/virtual_path
    ${PROJECT_SOURCE_DIR}/include/obstacle_detector
    ${EIGEN3_INCLUDE_DIRS}
    )
#include_directories(${JSON_INC_PATH})
add_definitions(${EIGEN3_DEFINITIONS})

add_library(${library_name} SHARED
  src/virtual_path/virtual_path_manager.cpp
  src/virtual_path/virtual_path_helper.cpp
  src/path_planner/follow_path_planner.cpp
  src/controller/pid_controller.cpp
  src/controller/pid.cpp
  src/manager/follow_path_manager.cpp
  src/obstacle_detector/obstacle_detector.cpp
)

aux_source_directory(./src SRC_LIST)

target_link_libraries(qrb_follow_path_manager
  tinyxml2
)

ament_target_dependencies(qrb_follow_path_manager

)

set_target_properties(${PROJECT_NAME} PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION 1
)

install(TARGETS qrb_follow_path_manager
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

file(GLOB_RECURSE HEADER_FILES
  "${CMAKE_SOURCE_DIR}/include/*.h"
  "${CMAKE_SOURCE_DIR}/include/*.hpp"
)

install(FILES ${HEADER_FILES}
  DESTINATION include/${PROJECT_NAME}/
)

install(DIRECTORY
  include
  DESTINATION include/${PROJECT_NAME}/
)

ament_auto_package()
