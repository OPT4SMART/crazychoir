import os
import pathlib

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from webots_ros2_driver.webots_launcher import WebotsLauncher

node_frequency = 100 # [Hz]

def get_webots_driver_cf(agent_id):
    package_dir_driver = get_package_share_directory('crazychoir_examples')
    robot_description = pathlib.Path(os.path.join(package_dir_driver, 'crazyflie_fpqr.urdf')).read_text()
    crazyflie_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        namespace=f'agent_{agent_id}',
        output='screen',
        additional_env={
            'WEBOTS_ROBOT_NAME':f'agent_{agent_id}',
            },
        parameters=[
            {'robot_description': robot_description},
        ]
    )

    return crazyflie_driver

def generate_webots_world_file(robots, source_filename, target_filename):
    with open(source_filename, 'r') as source_file:
        contents = source_file.read()

    with open(target_filename, 'w') as target_file:
        target_file.write(contents)

        for robot in robots:
            template_filename = os.path.join(os.path.dirname(source_filename), f'obj_{robot["type"]}.wbt')
            with open(template_filename, 'r') as template_file:
                template = template_file.read()
                template = template.replace('$NAME', robot["name"])
                template = template.replace('$X', str(robot["position"][0]))
                template = template.replace('$Y', str(robot["position"][1]))
                template = template.replace('$Z', str(robot["position"][2]))
                target_file.write(template)


def generate_launch_description():

    initial_position = [0.0, 0.0, 0.015]
    robots = [{
                'name': 'agent_0',
                'type': 'crazyflie', 
                'position': initial_position, 
            }]
    

    # initialize launch description
    launch_description = []

    world_package_dir = get_package_share_directory('crazychoir_examples')
    source_filename = os.path.join(world_package_dir, 'worlds', 'empty_world.wbt')
    target_filename = os.path.join(world_package_dir, 'worlds', 'my_world.wbt')
    generate_webots_world_file(robots, source_filename, target_filename)            
    webots = WebotsLauncher(world=target_filename)
    launch_description.append(webots)


    # Launch control Panel
    launch_description.append(Node(
                package='crazychoir_examples', 
                executable='crazychoir_tracking_webots_gui',
                output='screen',
                parameters=[{
                    'n_agents': 1,
                    }]))
            
    # webots exec
    launch_description.append(get_webots_driver_cf(0))
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
