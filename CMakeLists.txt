cmake_minimum_required(VERSION 3.8)
project(neo_linear_axis)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

add_library(neo_linear_axis 
  SHARED
  src/ElmoMotorModule.cpp
  src/nla_hardware_interface.cpp
  src/neo_linear_joint.cpp)
target_include_directories(neo_linear_axis PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(neo_linear_axis PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  neo_linear_axis
  "geometry_msgs"
  "std_msgs"
  "rclcpp"
  "trajectory_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "sensor_msgs"
  "builtin_interfaces"
  "rclcpp_lifecycle"
  "hardware_interface"
  "pluginlib"
)

pluginlib_export_plugin_description_file(
  hardware_interface nla.xml)

install(TARGETS neo_linear_axis
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

install(
  DIRECTORY include/
  DESTINATION include
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

ament_package()
