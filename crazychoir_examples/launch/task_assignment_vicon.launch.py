
import numpy as np

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

from disropt.utils.graph_constructor import binomial_random_graph

frequency = 100 # [Hz]

def generate_launch_description():

    # Set uri address for each quadrotor
    uris = [
        'radio://0/80/2M/E7E7E7E701',
        'radio://0/80/2M/E7E7E7E702',
        'radio://1/90/2M/E7E7E7E703',
        'radio://1/90/2M/E7E7E7E704',
        'radio://2/100/2M/E7E7E7E705',
        'radio://2/100/2M/E7E7E7E706',
    ]

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
    radio_launch = []

    # Launch control Panel
    launch_description.append(Node(
                package='crazychoir_examples', 
                executable='crazychoir_task_assignment_vicon_gui',
                output='screen',
                parameters=[{
                    'n_agents': N,
                    }]))
    
    # Launch radio node
    radios = set([int(uri.split('/')[2]) for uri in uris])
    for r in radios:
        uris_r = [uri for uri in uris if int(uri.split('/')[2]) == r]
        agent_ids_r = [uris.index(uri_r) for uri_r in uris_r]
        radio_launch.append(Node(
            package='crazychoir_examples', 
            executable='crazychoir_task_assignment_vicon_radio', 
            output='screen',
            namespace='radio_{}'.format(r),
            parameters=[{
                'uris': uris_r,
                'agent_ids': agent_ids_r,
                'cmd_topic': 'traj_params',
                }]))    
        
    # Launch vicon node
    launch_description.append(Node(
            package='vicon_receiver', 
            executable='vicon_client', 
            output='screen',
            parameters=[{
                'hostname': '192.168.10.1', 
                'buffer_size': 200, 
                'namespace': 'vicon'}]
        ))     
       
    # add task table executable
    launch_description.append(Node(
            package='crazychoir_examples', 
            executable='crazychoir_task_assignment_vicon_table', 
            output='screen',
            prefix='xterm -title "Table" -hold -e',
            parameters=[{'N': N}]))

    # add executables for each robot
    for i in range(N):

        uri = uris[i]
        in_neighbors  = np.nonzero(Adj[:, i])[0].tolist()
        out_neighbors = np.nonzero(Adj[i, :])[0].tolist()
        initial_position = P[i, :].tolist()

        # guidance
        launch_description.append(Node(
            package='crazychoir_examples', 
            executable='crazychoir_task_assignment_vicon_guidance', 
            output='screen',
            namespace='agent_{}'.format(i),
            parameters=[{
                'agent_id': i, 
                'N': N, 
                'in_neigh': in_neighbors, 
                'out_neigh': out_neighbors,
                'vicon_id': uri[-1],
                }]))

        # simple guidance (takeoff and landing)
        launch_description.append(Node(
            package='crazychoir_examples', 
            executable='crazychoir_task_assignment_vicon_simple_guidance', 
            output='screen',
            namespace='agent_{}'.format(i),
            parameters=[{
                'agent_id': i, 
                'vicon_id': uri[-1],
                'init_pos': initial_position,
                }]))
        
        # planner
        launch_description.append(Node(
            package='crazychoir_examples', 
            executable='crazychoir_task_assignment_vicon_planner', 
            output='screen',
            namespace='agent_{}'.format(i),
            parameters=[{
                'agent_id': i,
                'vicon_id': uri[-1],
                'freq': frequency,                       
                }]))
        
    # include delayed radio launcher
    timer_action = TimerAction(period=5.0, actions=[LaunchDescription(radio_launch)])
    launch_description.append(timer_action)

    return LaunchDescription(launch_description)
