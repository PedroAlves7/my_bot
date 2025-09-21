cmake_minimum_required(VERSION 3.8)
project(my_bot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find necessary dependencies for the C++ plugin
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(controller_manager REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(joint_state_broadcaster REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ros2_control REQUIRED)

# Add your C++ plugin's source files here
# This command tells CMake to compile a shared library from your source code
# You must have a 'src' directory in your package with a C++ file inside
add_library(my_bot_gazebo_plugin SHARED src/my_bot_gazebo_plugin.cpp)

# Link the dependencies to your plugin library
ament_target_dependencies(my_bot_gazebo_plugin
  rclcpp
  hardware_interface
  gazebo_ros
)

# This command exports the plugin description so Gazebo can find it
pluginlib_export_plugin_description_file(hardware_interface
  ${CMAKE_CURRENT_SOURCE_DIR}/my_bot_plugin.xml
)

# Install the compiled C++ library to the 'lib' directory
install(
  TARGETS
    my_bot_gazebo_plugin
  DESTINATION
    lib
)

# Install other files, like the URDFs, launch files, and configs
install(
  DIRECTORY
    launch config description worlds
  DESTINATION
    share/${PROJECT_NAME}
)

ament_package()