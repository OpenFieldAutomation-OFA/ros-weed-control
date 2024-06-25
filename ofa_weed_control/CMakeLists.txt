cmake_minimum_required(VERSION 3.8)
project(ofa_weed_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  moveit_ros_planning_interface
  moveit_visual_tools
  rclcpp
  tf2
  # tf2_ros
)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
# add_executable(arm_moveit src/tf2_test.cpp)
add_executable(arm_moveit src/arm_moveit.cpp)
target_include_directories(arm_moveit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(arm_moveit PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  arm_moveit
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

## INSTALL
install(
  TARGETS arm_moveit
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
