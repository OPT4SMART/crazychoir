import os
import pathlib

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from webots_ros2_driver.webots_launcher import WebotsLauncher

node_frequency = 100 # [Hz]

package_dir = get_package_share_directory('crazychoir_examples')

def generate_launch_description():

    initial_position = [0.0, 0.0, 0.015]

    # initialize launch description
    launch_description = []

    # Launch control Panel
    launch_description.append(Node(
                package='crazychoir_examples', 
                executable='crazychoir_tracking_webots_gui',
                output='screen',
                parameters=[{
                    'n_agents': 1,
                    }]))
            
    webots = WebotsLauncher(world=os.path.join(package_dir, 'worlds', 'tracking_world.wbt'))

    launch_description.append(webots)

    # webots exec
    robot_description = pathlib.Path(os.path.join(package_dir, 'crazyflie_fpqr.urdf')).read_text()
    launch_description.append(Node(
        package='webots_ros2_driver',
        executable='driver',
        namespace='agent_0',
        output='screen',
        additional_env={
            'WEBOTS_ROBOT_NAME':'agent_0',},
        parameters=[
            {'robot_description': robot_description},
            ]))
    launch_description.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        additional_env={'WEBOTS_ROBOT_NAME':'agent_0'},
        namespace='agent_0',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>',
            }]))

    # controller
    launch_description.append(Node(
        package='crazychoir_examples', 
        executable='crazychoir_tracking_webots_controller', 
        namespace='agent_0',
        output='screen',
        parameters=[{
            'agent_id': 0,
            }]))

    # guidance
    launch_description.append(Node(
        package='crazychoir_examples', 
        executable='crazychoir_tracking_webots_guidance', 
        namespace='agent_0',
        output='screen',
        parameters=[{
            'agent_id': 0,
            'init_pos': initial_position,
            }]))


    # reference
    launch_description.append(Node(
        package='crazychoir_examples',
        executable='crazychoir_tracking_webots_trajectory', 
        namespace='agent_0',
        output='screen',
        parameters=[{     
            'agent_id': 0, 
            }]))

    return LaunchDescription(launch_description)
