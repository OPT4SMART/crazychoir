import os
import numpy as np
import pathlib

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from webots_ros2_driver.webots_launcher import WebotsLauncher

node_frequency = 100 # [Hz]

package_dir = get_package_share_directory('crazychoir_examples')

def get_cf_driver(agent_id):
    robot_description = pathlib.Path(os.path.join(package_dir, 'crazyflie_fpqr.urdf')).read_text()

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
    N = 11

    # communication matrix
    Adj = np.array([
        [0,	1,	1,	1,	1,	0,	0,	0,	1,	0,	0],
        [1,	0,	1,	1,	1,	0,	0,	1,	0,	0,	0],
        [1,	1,	0,	1,	1,	0,	0,	0,	0,	0,	0],
        [1,	1,	1,	0,	1,	1,	1,	0,	0,	0,	1],
        [1,	1,	1,	1,	0,	1,	0,	0,	0,	1,	1],
        [0,	0,	0,	1,	1,	0,	1,	0,	1,	1,	1],
        [0,	0,	0,	1,	0,	1,	0,	1,	1,	1,	1],
        [0,	1,	0,	0,	0,	0,	1,	0,	1,	1,	1],
        [1,	0,	0,	0,	0,	1,	1,	1,	0,	1,	0],
        [0,	0,	0,	0,	1,	1,	1,	1,	1,	0,	1],
        [0,	0,	0,	1,	1,	1,	1,	1,	0,	1,	0],
    ])

    # generate initial positions to evaluate initial takeoff
    P = np.zeros((N, 3))   
    P[0]  = np.array([ 0.5508979 ,  0.87484782,  0.015])
    P[1]  = np.array([-0.70899526, -0.65577239,  0.015])
    P[2]  = np.array([ 0.39294695,  0.39619309,  0.015])
    P[3]  = np.array([ 0.32558531, -0.62615712,  0.015])
    P[4]  = np.array([ 0.0513672 , -0.72589016,  0.015])
    P[5]  = np.array([-0.10322379, -0.94286678,  0.015])
    P[6]  = np.array([-0.01725595, -1.22131272,  0.015])
    P[7]  = np.array([-0.3234451 ,  0.42256282,  0.015])
    P[8]  = np.array([-0.97551812,  0.72535409,  0.015])
    P[9]  = np.array([-0.40714755,  0.9138012 ,  0.015])
    P[10] = np.array([-0.04957492,  1.19173792,  0.015])

    # generate coordinates to evaluate desired bearings
    D = np.zeros((N, 3))
    D[0]  = np.array([  0.5001,  0.6667, 0.5])
    D[1]  = np.array([ -0.4999, -0.6666, 0.5])
    D[2]  = np.array([ -0.0000, -0.0001, 0.5])
    D[3]  = np.array([  0.5000, -0.3334, 0.5])
    D[4]  = np.array([  0.4999, -0.6667, 0.5])
    D[5]  = np.array([  0.1669, -0.9997, 0.5])
    D[6]  = np.array([ -0.1664, -0.9998, 0.5])
    D[7]  = np.array([ -0.4997,  0.3317, 0.5])
    D[8]  = np.array([ -0.4995,  0.6665, 0.5])
    D[9]  = np.array([ -0.1664,  0.9987, 0.5])
    D[10] = np.array([  0.1669,  0.9986, 0.5])

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

    # Launch control Panel
    launch_description.append(Node(
                package='crazychoir_examples', 
                executable='crazychoir_formation_webots_gui',
                output='screen',
                parameters=[{
                    'n_agents': N,
                    }]))

            
    webots = WebotsLauncher(world=os.path.join(package_dir, 'worlds', 'letter_formation_world.wbt'))

    launch_description.append(webots)

    # add executables for each robot
    for i in range(N):

        in_neighbors  = np.nonzero(Adj[:, i])[0].tolist()
        out_neighbors = np.nonzero(Adj[i, :])[0].tolist()
        init_pos = P[i, :].tolist()
        is_leader = True if i < 2 else False
        orth_proj_array = orth_proj[i*dd:(i+1)*dd,:].flatten().tolist()

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
            executable='crazychoir_formation_webots_guidance', 
            output='screen',
            namespace='agent_{}'.format(i),
            parameters=[{
                'freq': node_frequency, 
                'agent_id': i, 
                'N': N, 
                'dd': dd, 
                'in_neigh': in_neighbors, 
                'out_neigh': out_neighbors, 
                'is_leader': is_leader, 
                'init_pos': init_pos,
                'ort_proj': orth_proj_array}]))

        # controller
        if is_leader:
            controller_str = "leaders"
        else:
            controller_str = "followers"
            
        launch_description.append(Node(
            package='crazychoir_examples', 
            executable='crazychoir_formation_webots_controller_{}'.format(controller_str), 
            namespace='agent_{}'.format(i),
            output='screen',
            parameters=[{
                'freq': node_frequency,                       
                'agent_id': i,
                'is_leader': is_leader, 
                }]))

        # reference
        if is_leader:
            launch_description.append(Node(
                package='crazychoir_examples',
                executable='crazychoir_formation_webots_trajectory', 
                namespace='agent_{}'.format(i),
                output='screen',
                parameters=[{     
                    'freq': node_frequency,                       
                    'agent_id': i, 
                    }]))

    return LaunchDescription(launch_description)
