import numpy as np

from launch import LaunchDescription
from launch_ros.actions import Node
from disropt.utils.graph_constructor import binomial_random_graph, ring_graph, metropolis_hastings
from launch.actions import TimerAction


freq_guidance = 5  # [Hz]
freq_reference = 10    # [Hz]
freq_controller = 10   # [Hz]


def generate_launch_description():

    # Set uri address for each quadrotor
    uris = [
        'radio://0/100/2M/E7E7E7E700',
        'radio://0/100/2M/E7E7E7E701',
        'radio://0/100/2M/E7E7E7E702'
    ]

    N = 3
    max_iter = 20000


    # generate communication graph (this function also sets the seed)
    Adj = binomial_random_graph(N, p=0.5, seed=3)
    W = metropolis_hastings(Adj)

    P = np.zeros((N, 3))
    P[0] = np.array([-0.5, -0.5, 0.0])
    P[1] = np.array([ 0.0, -0.5, 0.0])
    P[2] = np.array([ 0.5, -0.5, 0.0])

    # Target
    target = [0.0, 0.0, 1.0]

    # Intruders
    intruders = np.zeros((N, 3))
    intruders[0] = np.array([-1.0,  1.0, 1.0])
    intruders[1] = np.array([-1.0, -1.0, 1.0])
    intruders[2] = np.array([ 1.0,  0.0, 1.0])


    launch_description = []
    radio_launch = []

    # Launch control Panel
    launch_description.append(Node(
                package='crazychoir_examples', 
                executable='crazychoir_aggregative_lighthouse_gui',
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
            executable='crazychoir_aggregative_lighthouse_radio', 
            prefix='xterm -title "radio_{}" -hold -e'.format(0),
            namespace='radio_{}'.format(r),
            parameters=[{
                'uris': uris_r,
                'agent_ids': agent_ids_r,
                'cmd_topic': 'cmd_vel',
                'logger' : True,
                }]))  

    # Add executables for each robot
    for i in range(N):
        initial_position = P[i, :].tolist()
    
        # Guidance
        in_neighbors  = np.nonzero(Adj[:, i])[0].tolist()
        out_neighbors = np.nonzero(Adj[i, :])[0].tolist()
        weights = W[i,:].tolist()
        launch_description.append(Node(
            package='crazychoir_examples',
            executable='crazychoir_aggregative_lighthouse_guidance_aggregative', 
            output='screen',
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
                'intruder': intruders[i].tolist(),
                }]))
            
            
        # Simple guidance (takeoff and landing)
        launch_description.append(Node(
            package='crazychoir_examples', 
            executable='crazychoir_aggregative_lighthouse_simple_guidance', 
            output='screen',
            namespace=f'agent_{i}',
            parameters=[{
                'agent_id': i, 
                'init_pos': initial_position,
                }]))

        # Controller
        launch_description.append(Node(
            package='crazychoir_examples', 
            executable='crazychoir_aggregative_lighthouse_controller', 
            namespace=f'agent_{i}',
            output='screen',
            parameters=[{
                'freq': freq_controller,                       
                'agent_id': i,
                }]))
        
        # Trajectory
        launch_description.append(Node(
            package='crazychoir_examples',
            executable='crazychoir_aggregative_lighthouse_trajectory', 
            namespace=f'agent_{i}',
            output='screen',
            parameters=[{     
                'freq': freq_reference,                       
                'agent_id': i, 
                }]))
    
    # include delayed radio launcher
    timer_action = TimerAction(period=1.0, actions=[LaunchDescription(radio_launch)])
    launch_description.append(timer_action)

    return LaunchDescription(launch_description)
