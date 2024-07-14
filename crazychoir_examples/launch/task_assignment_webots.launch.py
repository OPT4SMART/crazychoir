
import os
import numpy as np
import pathlib

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from disropt.utils.graph_constructor import binomial_random_graph
from webots_ros2_driver.webots_launcher import WebotsLauncher

frequency = 100 # [Hz]

def get_webots_driver_cf(agent_id):
    package_dir_driver = get_package_share_directory('crazychoir_examples')
    robot_description = pathlib.Path(os.path.join(package_dir_driver, 'crazyflie_xyz.urdf')).read_text()
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
    N = 6

    # generate communication graph (this function also sets the seed)
    Adj = binomial_random_graph(N, 0.2, seed=3)

    # generate initial positions in [-3, 3] with z = 0
    P = np.zeros((N, 3))
    P[0]  = np.array([ 1.0, 1.0, 0.015])
    P[1]  = np.array([ 0.0, 1.5, 0.015])
    P[2]  = np.array([-1.0, 1.0, 0.015])
    P[3]  = np.array([-1.0,-1.0, 0.015])
    P[4]  = np.array([ 0.0,-1.5, 0.015])
    P[5]  = np.array([ 1.0,-1.0, 0.015])

    # initialize launch description
    launch_description = [] # launched immediately 


    # Generate Webots world
    robots = [{
                'name': f'agent_{i}',
                'type': 'crazyflie', 
                'position': P[i, :].tolist(),
            } for i in range(N)]
    
    world_package_dir = get_package_share_directory('crazychoir_examples')
    source_filename = os.path.join(world_package_dir, 'worlds', 'empty_world.wbt')
    target_filename = os.path.join(world_package_dir, 'worlds', 'my_world.wbt')
    generate_webots_world_file(robots, source_filename, target_filename)            
    webots = WebotsLauncher(world=target_filename)
    launch_description.append(webots)


    # Launch control Panel
    launch_description.append(Node(
                package='crazychoir_examples', 
                executable='crazychoir_task_assignment_webots_gui',
                output='screen',
                parameters=[{
                    'n_agents': N,
                    }]))

    # add task table executable
    launch_description.append(Node(
            package='crazychoir_examples', 
            executable='crazychoir_task_assignment_webots_table', 
            output='screen',
            prefix='xterm -title "Table" -hold -e',
            parameters=[{'N': N}]))


    # add executables for each robot
    for i in range(N):

        in_neighbors  = np.nonzero(Adj[:, i])[0].tolist()
        out_neighbors = np.nonzero(Adj[i, :])[0].tolist()
        initial_position = P[i, :].tolist()

        # webots exec
        launch_description.append(get_webots_driver_cf(i))
        launch_description.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            additional_env={'WEBOTS_ROBOT_NAME':'agent_{}'.format(i)},
            namespace='agent_{}'.format(i),
            output='screen',
            parameters=[{
                'robot_description': '<robot name=""><link name=""/></robot>',
                }]))

        # guidance
        launch_description.append(Node(
            package='crazychoir_examples', 
            executable='crazychoir_task_assignment_webots_guidance', 
            output='screen',
            namespace='agent_{}'.format(i),
            parameters=[{
                'agent_id': i, 
                'N': N, 
                'in_neigh': in_neighbors, 
                'out_neigh': out_neighbors
                }]))

        # simple guidance (takeoff and landing)
        launch_description.append(Node(
            package='crazychoir_examples', 
            executable='crazychoir_task_assignment_webots_simple_guidance', 
            output='screen',
            namespace='agent_{}'.format(i),
            parameters=[{
                'agent_id': i, 
                'init_pos': initial_position,
                }]))
        
        # planner
        launch_description.append(Node(
            package='crazychoir_examples', 
            executable='crazychoir_task_assignment_webots_planner', 
            output='screen',
            namespace='agent_{}'.format(i),
            parameters=[{
                'agent_id': i,
                'freq': frequency,                       
                }]))
        
    
    return LaunchDescription(launch_description)
