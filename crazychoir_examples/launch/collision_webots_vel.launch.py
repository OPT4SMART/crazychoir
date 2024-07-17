import os
import numpy as np
import pathlib

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from webots_ros2_driver.webots_launcher import WebotsLauncher

node_frequency = 10 # [Hz]



def get_webots_driver_cf(agent_id):
    package_dir_driver = get_package_share_directory('crazychoir_examples')
    robot_description = pathlib.Path(os.path.join(package_dir_driver, 'crazyflie_vel.urdf')).read_text()
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

    # number of agents
    N = 2

    # generate initial locations
    P = np.array([[-1.0, 0.1, 0.0],
                  [ 1.0,-0.1, 0.0],
                  ])
    robots = [{
                'name': f'agent_{i}',
                'type': 'crazyflie', 
                'position': P[i, :].tolist(), 
            } for i in range(N) ]
    
    # generate goals locations {i: [[time, x, y, z],[...]]}
    goals = {
        0: [[5, -1.0, 0.1, 1.0],[20, 1.0, 0.1, 1.0]],
        1: [[5,  1.0,-0.1, 1.0],[20,-1.0,-0.1, 1.0]],
    }


    robots +=[{
            'name': f'target_{i}',
            'type': 'target', 
            'position': goals[i][1][1:],
        } for i in range(N)]

    # initialize launch description
    launch_description = []

    # Launch control Panel
    launch_description.append(Node(
                package='crazychoir_examples', 
                executable='crazychoir_collision_webots_gui',
                output='screen',
                parameters=[{
                    'n_agents': N,
                    }]))

            
    # Generate Webots World
    world_package_dir = get_package_share_directory('crazychoir_examples')
    source_filename = os.path.join(world_package_dir, 'worlds', 'empty_world.wbt')
    target_filename = os.path.join(world_package_dir, 'worlds', 'my_world.wbt')
    generate_webots_world_file(robots, source_filename, target_filename)            
    webots = WebotsLauncher(world=target_filename)
    launch_description.append(webots)
    
    # neighbors getter
    launch_description.append(Node(
        package='crazychoir_examples', executable='crazychoir_collision_webots_closest_robot_getter', output='screen', 
        prefix='xterm  -title "neigh_getter" -geometry 160x7+1980+0 -hold -e ',# if i == 0 else None,
        parameters=[{'first_id': 0, 'last_id':N, 'freq': 100.0}]))

    # add executables for each robot
    for i in range(N):

        init_pos = P[i, :].tolist()
        goals_i = [item for goal in goals[i] for item in goal]

        # webots exec
        launch_description.append(get_webots_driver_cf(i))
        launch_description.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            additional_env={'WEBOTS_ROBOT_NAME':f'agent_{i}'},
            namespace=f'agent_{i}',
            output='screen',
            parameters=[{
                'robot_description': '<robot name=""><link name=""/></robot>',
                }]))

        # guidance
        launch_description.append(Node(
            package='crazychoir_examples',
            executable='crazychoir_collision_webots_guidance', 
            output='screen',
            prefix=f'xterm  -title "guid_{i}" -geometry 160x7+1980+{int(135*(i+1))} -hold -e ',# if i == 0 else None,
            namespace=f'agent_{i}',
            parameters=[{
                'freq': node_frequency, 
                'agent_id': i, 
                'N': N, 
                'init_pos': init_pos,
                'goals': goals_i}]))

        # controller
        launch_description.append(Node(
            package='crazychoir_examples', 
            executable='crazychoir_collision_webots_controller_vel', 
            namespace=f'agent_{i}',
            output='screen',
            prefix=f'xterm  -title "contr_{i}" -geometry 160x7+1980+{int(135*(i+1+N))} -hold -e ',# if i == 0 else None,
            remappings=[('cmd_vel', 'cmd_vel_des')],
            parameters=[{
                'freq': node_frequency,                       
                'agent_id': i,
                'N': N,
                }]))
        
        
        # collision avoidance
        launch_description.append(Node(
            package='crazychoir_examples', executable='crazychoir_collision_webots_collision', output='screen', 
            prefix=f'xterm  -title "coll_{i}" -geometry 160x7+1980+{int(135*(i+1))+1225} -hold -e ',# if i == 0 else None,
            namespace=f'agent_{i}',
            parameters=[{'agent_id': i, 'N':N, 'freq': 50.0}]))


        # reference
        launch_description.append(Node(
            package='crazychoir_examples',
            executable='crazychoir_collision_webots_trajectory_vel', 
            namespace=f'agent_{i}',
            output='screen',
            parameters=[{     
                'freq': node_frequency,                       
                'agent_id': i, 
                }]))
        
    return LaunchDescription(launch_description)
