#!/bin/bash

DEV_NAME=opt4smart
ROS_DISTRO=foxy
WS_NAME=crazychoir_ws


cd /home/${DEV_NAME}/${WS_NAME}/src/ros2-vicon-receiver 
./install_libs.sh

# Clean up unnecessary webots_ros2 packages
rm -rf /home/${DEV_NAME}/${WS_NAME}/src/webots_ros2/webots_ros2_core \
           /home/${DEV_NAME}/${WS_NAME}/src/webots_ros2/webots_ros2_epuck \
           /home/${DEV_NAME}/${WS_NAME}/src/webots_ros2/webots_ros2_mavic \
           /home/${DEV_NAME}/${WS_NAME}/src/webots_ros2/webots_ros2_tesla \
           /home/${DEV_NAME}/${WS_NAME}/src/webots_ros2/webots_ros2_tests \
           /home/${DEV_NAME}/${WS_NAME}/src/webots_ros2/webots_ros2_tiago \
           /home/${DEV_NAME}/${WS_NAME}/src/webots_ros2/webots_ros2_turtlebot \
           /home/${DEV_NAME}/${WS_NAME}/src/webots_ros2/webots_ros2_universal_robot

sed -i "s@/PATH/TO/crazyflie-firmware@/home/opt4smart/crazyflie-firmware/build@g" /home/${DEV_NAME}/${WS_NAME}/src/crazychoir/crazychoir/utils/__init__.py