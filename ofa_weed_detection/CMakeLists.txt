cmake_minimum_required(VERSION 3.8)
project(ofa_weed_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  orbbec_camera
)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_executable(process_image src/process_image.cpp)
target_include_directories(process_image PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_SOURCE_DIR}/sdk/include
  ${CMAKE_CURRENT_SOURCE_DIR}/sdk/include/inc-pub)
target_compile_features(process_image PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  process_image
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
target_link_libraries(process_image ${CMAKE_CURRENT_SOURCE_DIR}/sdk/lib/libsFoundation20.so)

## INSTALL
install(
  TARGETS process_image
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
