import numpy as np

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

node_frequency = 100 # [Hz]

def generate_launch_description():

    # Set uri address for each quadrotor
    uris = [
        'radio://0/80/2M/E7E7E7E701',
        'radio://1/90/2M/E7E7E7E702',
        'radio://2/100/2M/E7E7E7E703',
        'radio://3/110/2M/E7E7E7E704',
    ]

    # number of agents
    N = 4

    # communication matrix
    Adj = np.array([
        [0, 1, 0, 1],
        [1, 0, 1, 1],
        [0, 1, 0, 1],
        [1, 1, 1, 0],
    ])

    # generate initial positions to evaluate initial takeoff
    P = np.zeros((N, 3))   
    P[0] = np.array([-0.7, -0.7, 0.015])
    P[1] = np.array([ 0.7, -0.7, 0.015])
    P[2] = np.array([ 0.1,  0.1, 0.015])
    P[3] = np.array([-0.6,  0.3, 0.015])

    # generate coordinates to evaluate desired bearings
    D = np.zeros((N, 3))
    D[0,0:3] = np.array([-1, -1, 1.0])*0.7
    D[1,0:3] = np.array([ 1, -1, 1.0])*0.7
    D[2,0:3] = np.array([ 1,  1, 1.0])*0.7
    D[3,0:3] = np.array([-1,  1, 1.0])*0.7

    dd = np.size(D,1)

    orth_proj = np.zeros((dd*N, dd*N))
    for i in range(N):
        neigh_ii = np.nonzero(Adj[i, :])[0].tolist()
        for j in neigh_ii:
            x_des_i = D[i, :]
            x_des_j = D[j, :]
            # compute desired formation_webots
            g_ij = np.matrix((x_des_j - x_des_i)/np.linalg.norm(x_des_j - x_des_i))
            # fill the matrix
            orth_proj[i*dd:(i+1)*dd,j*dd:(j+1)*dd] = np.eye(dd) - g_ij.T @ g_ij

    # initialize launch description
    launch_description = []
    radio_launch = []

    # Launch control Panel
    launch_description.append(Node(
                package='crazychoir_examples', 
                executable='crazychoir_formation_vicon_gui',
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
            executable='crazychoir_formation_vicon_radio', 
            output='screen',
            namespace='radio_{}'.format(r),
            prefix='xterm -title "radio_{}" -hold -e'.format(r),
            parameters=[{
                'uris': uris_r,
                'agent_ids': agent_ids_r,
                'cmd_topic': 'cmd_vel',
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

    # add executables for each robot
    for i in range(N):
        
        uri = uris[i]
        in_neighbors  = np.nonzero(Adj[:, i])[0].tolist()
        out_neighbors = np.nonzero(Adj[i, :])[0].tolist()
        init_pos = P[i, :].tolist()
        is_leader = True if i < 2 else False
        orth_proj_array = orth_proj[i*dd:(i+1)*dd,:].flatten().tolist()

        # guidance
        launch_description.append(Node(
            package='crazychoir_examples',
            executable='crazychoir_formation_vicon_guidance', 
            prefix='xterm -title "guidance_{}" -hold -e'.format(i),
            output='screen',
            namespace='agent_{}'.format(i),
            parameters=[{
                'freq': node_frequency, 
                'agent_id': i,
                'N': N, 
                'in_neigh': in_neighbors, 
                'out_neigh': out_neighbors, 
                'vicon_id': uri[-1],
                'vicon': True,
                'dd': dd, 
                'is_leader': is_leader, 
                'init_pos': init_pos,
                'ort_proj': orth_proj_array}]))

        # controller
        launch_description.append(Node(
            package='crazychoir_examples', 
            executable='crazychoir_formation_vicon_controller', 
            namespace='agent_{}'.format(i),
            prefix='xterm -title "controller_{}" -hold -e'.format(i),
            output='screen',
            parameters=[{
                'freq': node_frequency,                       
                'vicon_id': uri[-1],
                'vicon': True,
                'agent_id': i,
                'is_leader': is_leader, 
                }]))

        # reference
        if is_leader:
            launch_description.append(Node(
                package='crazychoir_examples',
                executable='crazychoir_formation_vicon_trajectory', 
                namespace='agent_{}'.format(i),
                output='screen',
                parameters=[{     
                    'freq': node_frequency,                       
                    'vicon': True,
                    'vicon_id': uri[-1],
                    'agent_id': i, 
                    }]))

    # include delayed radio launcher
    timer_action = TimerAction(period=5.0, actions=[LaunchDescription(radio_launch)])
    launch_description.append(timer_action)

    return LaunchDescription(launch_description)
