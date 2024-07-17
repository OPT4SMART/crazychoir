import os
import numpy as np
import pathlib
import sys

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from disropt.utils.graph_constructor import binomial_random_graph, ring_graph, metropolis_hastings


freq_guidance = 5  # [Hz]
freq_reference = 10    # [Hz]
freq_controller = 10   # [Hz]


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

    N = 3
    max_iter = 20000

    # generate communication graph (this function also sets the seed)
    Adj = binomial_random_graph(N, p=0.5, seed=3)
    W = metropolis_hastings(Adj)

    P = np.zeros((2*N, 3))

    # Turltebots init positions
    P[0] = np.array([ 0.5, -0.2, 0.0])
    P[1] = np.array([-0.3, -0.5, 0.0])
    P[2] = np.array([-0.1,  0.5, 0.0])
    P[3] = np.array([-1.0, -1.0, 0.0])
    P[4] = np.array([-1.0,  1.0, 0.0])
    P[5] = np.array([ 1.0,  1.0, 0.0])

    robots = [{
                'name': f'agent_{i}',
                'type': 'crazyflie', 
                'position': P[i, :].tolist(), 
            } for i in range(N) ]

    robots += [{
                'name': f'agent_{i}',
                'type': 'crazyflie_red', 
                'position': P[i, :].tolist(), 
            } for i in range(N,2*N) ]

    # Target
    target = [0.0, 0.0, 1.0]
    robots +=[{
                'name': 'target',
                'type': 'target', 
                'position': target, 
            }]


    intruders_goals = {
    # id: [[t, x, y, z], ...]
        3: [[5,-1.0, -1.0, 1.0], [20,-0.9,  0.9, 1.0], [20, 0.75,  0.75, 1.0]],
        4: [[5,-1.0,  1.0, 1.0], [20, 0.9,  0.9, 1.0], [20, 0.75, -0.75, 1.0]],
        5: [[5, 1.0,  1.0, 1.0], [20, 0.9, -0.9, 1.0], [20,-0.75, -0.75, 1.0]],
    }


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
        executable='crazychoir_aggregative_webots_gui',
        output='screen',
        parameters=[{
            'n_agents': 2*N,
            }]))


    # add executables for each robot
    for i in range(2*N):
        initial_position = P[i, :].tolist()
    
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


        if i < N:
            # guidance
            in_neighbors  = np.nonzero(Adj[:, i])[0].tolist()
            out_neighbors = np.nonzero(Adj[i, :])[0].tolist()
            weights = W[i,:].tolist()
            launch_description.append(Node(
                package='crazychoir_examples',
                executable='crazychoir_aggregative_webots_guidance_aggregative_moving', 
                output='screen',
                # prefix=f'xterm  -title "guidance_{i}" -geometry 160x7+1980+{int(135*(i))} -hold -e ',# if i == 0 else None,
                namespace=f'agent_{i}',
                parameters=[{
                    'freq': freq_guidance, 
                    'agent_id': i, 
                    'N': N, 
                    'in_neigh': in_neighbors, 
                    'out_neigh': out_neighbors, 
                    'weights': weights, 
                    'init_pos': initial_position,
                    'freq_reference': freq_reference,
                    'max_iter': max_iter,
                    'target': target,
                    }]))
            
        else:
            goals_i = [item for goal in intruders_goals[i] for item in goal]
            # guidance_intruder
            launch_description.append(Node(
                package='crazychoir_examples',
                executable='crazychoir_aggregative_webots_guidance_intruder', 
                output='screen',
                # prefix=f'xterm  -title "guidance_{i}" -geometry 160x4+1980+{int(95*i)} -hold -e ',# if i == 0 else None,
                namespace=f'agent_{i}',
                parameters=[{
                    'freq': freq_guidance, 
                    'agent_id': i, 
                    'N': N, 
                    'init_pos': initial_position,
                    'goals': goals_i}]))
            
        # simple guidance (takeoff and landing)
        launch_description.append(Node(
            package='crazychoir_examples', 
            executable='crazychoir_aggregative_webots_simple_guidance', 
            output='screen',
            namespace=f'agent_{i}',
            parameters=[{
                'agent_id': i, 
                'init_pos': initial_position,
                }]))

        # controller
        launch_description.append(Node(
            package='crazychoir_examples', 
            executable='crazychoir_aggregative_webots_controller', 
            namespace=f'agent_{i}',
            output='screen',
            # prefix=f'xterm  -title "controller_{i}" -geometry 160x4+1980+{int(95*(i+N))} -hold -e ',# if i == 0 else None,
            parameters=[{
                'freq': freq_controller,                       
                'agent_id': i,
                }]))
        
        # trajectory
        launch_description.append(Node(
            package='crazychoir_examples',
            executable='crazychoir_aggregative_webots_trajectory', 
            namespace=f'agent_{i}',
            output='screen',
            parameters=[{     
                'freq': freq_reference,                       
                'agent_id': i, 
                }]))
        

    return LaunchDescription(launch_description)
