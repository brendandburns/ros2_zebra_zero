cmake_minimum_required(VERSION 3.8)
project(zebra_zero)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
  include
  /home/bburns/src/libnmc
  /opt/ros/humble/include
)

set(DEPS
  pluginlib
  rcpputils
  hardware_interface
  controller_interface
  realtime_tools
  trajectory_msgs
  rclcpp
)

foreach(Dependency IN ITEMS ${DEPS})
  find_package(${Dependency} REQUIRED)
endforeach()

#add_library(nmc
#            SHARED
#            libnmc/nmccom.cpp
#            libnmc/picio.cpp
#            libnmc/picservo.cpp
#            libnmc/picstep.cpp
#            libnmc/sio_util.cpp
#            )
add_library(nmc STATIC IMPORTED)
set_target_properties(nmc PROPERTIES IMPORTED_LOCATION /usr/local/lib/libnmc.so)

add_library(
    zebra_zero
    SHARED
    src/nmc.cpp
    src/servo.cpp
    src/hal.cpp
    src/hardware.cpp
    src/controller.cpp
)
ament_target_dependencies(zebra_zero PUBLIC ${DEPS})
target_link_libraries(zebra_zero PUBLIC nmc)

find_package(rclcpp REQUIRED)
add_executable(zebra_zero_node
               src/zebra_zero_node.cpp)
ament_target_dependencies(zebra_zero_node rclcpp)

# target_link_libraries(zebra_zero /home/bburns/src/libnmc/libnmc.a)

pluginlib_export_plugin_description_file(hardware_interface zebra_zero.xml)
pluginlib_export_plugin_description_file(controller_interface zebra_zero.xml)

install(TARGETS
  zebra_zero_node 
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config urdf
  DESTINATION share/zebra_zero
)

install(
  DIRECTORY include/
  DESTINATION include/zebra_zero
)

# install(
#  DIRECTORY description/launch description/ros2_control description/urdf
#  DESTINATION share/ros2_control_demo_example_7
#)

# install(
#     TARGETS send_trajectory
#     RUNTIME DESTINATION lib/ros2_control_demo_example_7
#)

install(TARGETS zebra_zero
  EXPORT zebra_zero
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(zebra_zero HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPS})
ament_package()
