
import os
import numpy as np
import pathlib

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from disropt.utils.graph_constructor import binomial_random_graph
from webots_ros2_driver.webots_launcher import WebotsLauncher

frequency = 100 # [Hz]

package_dir = get_package_share_directory('crazychoir_examples')

def get_cf_driver(agent_id):
    robot_description = pathlib.Path(os.path.join(package_dir, 'crazyflie_xyz.urdf')).read_text()

    crazyflie_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        namespace='agent_{}'.format(agent_id),
        output='screen',
        additional_env={
            'WEBOTS_ROBOT_NAME':'agent_{}'.format(agent_id),
            },
        parameters=[
            {'robot_description': robot_description},
        ]
    )

    return crazyflie_driver

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

    # Launch control Panel
    launch_description.append(Node(
                package='crazychoir_examples', 
                executable='crazychoir_formation_webots_gui',
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

    # Launch webots
    webots = WebotsLauncher(world=os.path.join(package_dir, 'worlds', 'task_assignment_world.wbt'))
    launch_description.append(webots)

    # add executables for each robot
    for i in range(N):

        in_neighbors  = np.nonzero(Adj[:, i])[0].tolist()
        out_neighbors = np.nonzero(Adj[i, :])[0].tolist()
        initial_position = P[i, :].tolist()

        # webots exec
        launch_description.append(get_cf_driver(i))
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
