cmake_minimum_required(VERSION 3.5)
project(crazychoir_interfaces)

# load requires packages
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs)

# generate C++ and Python classes
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FullState.msg"
  DEPENDENCIES geometry_msgs
)

# Install the python module for this package
ament_python_install_package(${PROJECT_NAME})

ament_export_dependencies(ament_cmake_python)

ament_package()
