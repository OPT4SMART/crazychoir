import os
import numpy as np
import pathlib
import networkx as nx
from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from webots_ros2_driver.webots_launcher import WebotsLauncher

node_frequency = 100 # [Hz]
np.random.seed(3)

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
    N_x = 5
    N_y = 6
    N = N_x*N_y

    # Communication matrix 
    Adj =np.array([[0,0,0,0,0,0,0,0,0,0,1,0,0,0,1,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0],[0,0,1,1,1,0,0,0,0,0,1,0,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0],[0,1,0,0,0,0,1,0,0,0,0,1,1,0,1,1,0,0,0,0,0,1,0,0,0,0,0,0,0,0],[0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,1,0,0,1,1,1,1,0,0,0,0,1,1],[0,1,0,0,0,0,0,0,0,1,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0],[0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,1,1,1,0,1,0,0,0,0],[0,0,0,1,0,0,0,0,1,1,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,1,0,1,1,0,0,1,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0],[0,0,0,0,1,0,0,1,1,0,0,0,1,0,1,1,0,1,0,0,1,1,0,0,0,0,0,0,1,0],[1,1,0,0,1,0,0,0,1,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0],[0,0,1,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,1],[0,0,1,0,0,1,0,0,0,1,0,0,0,1,1,0,0,1,0,1,0,0,0,0,0,0,0,0,0,0],[0,1,0,0,0,0,0,1,1,0,0,1,1,0,1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,0],[1,0,1,0,1,1,0,0,0,1,1,0,1,1,0,1,0,1,0,0,1,1,0,0,0,0,0,1,1,0],[0,0,1,1,0,1,0,0,0,1,0,0,0,0,1,0,0,0,1,1,1,0,0,0,1,0,0,0,1,0],[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,1],[0,1,0,1,0,0,0,0,0,1,0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1],[0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,1,0,0,0,0,1,1,0,0,1,1,0,1,0,0],[0,0,0,0,0,0,0,1,1,0,0,0,1,1,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0],[0,0,0,1,0,0,0,0,0,1,0,0,0,0,1,1,0,0,1,0,0,0,0,0,0,1,0,0,1,1],[0,0,1,1,0,0,1,0,0,1,0,0,0,0,1,0,1,0,1,0,0,0,0,0,1,1,1,1,0,0],[0,0,0,1,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],[1,0,0,1,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,1,0,0,1,0,0],[0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,1,0,0,1,0,0,1,0,1,0,0,0,0,0,1],[0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,0,1,1,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0],[0,1,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,1,0,0,1,0,1,0,0,0,0,0,0],[0,0,0,1,0,0,0,0,0,1,0,0,0,0,1,1,0,1,0,0,1,0,0,0,0,0,0,0,0,0],[0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,0,1,1,0,0,1,0,1,0,1,0,0,0,0,0],])
    
    # Initial Positions
    P = np.array([[-3.        , -3.        ,  0.015     ],[-1.5       , -3.        ,  0.015     ],[-0.16868619, -2.66125053,  0.015     ],[ 1.9132557 , -2.9219043 ,  0.015     ],[ 3.11907741, -3.39231912,  0.015     ],[-2.67833545, -1.81800163,  0.015     ],[-1.96498645, -1.48402579,  0.015     ],[ 0.20321469, -1.61266327,  0.015     ],[ 1.09590303, -1.73003791,  0.015     ],[ 3.1470463 , -1.79839019,  0.015     ],[-3.31201296, -0.87524828,  0.015     ],[-1.97213383, -0.64933408,  0.015     ],[-0.13868307, -0.37514833,  0.015     ],[ 1.50754652, -0.84702662,  0.015     ],[ 2.86333522, -0.27696913,  0.015     ],[-2.54766225,  0.6089557 ,  0.015     ],[-1.18429163,  1.02129148,  0.015     ],[ 0.05467076,  0.20579186,  0.015     ],[ 1.17376227,  0.50001821,  0.015     ],[ 2.75356153,  0.76464031,  0.015     ],[-2.79836865,  1.81598511,  0.015     ],[-1.12644963,  1.40757072,  0.015     ],[ 0.14037111,  1.47140232,  0.015     ],[ 1.82914217,  1.51191937,  0.015     ],[ 3.39199394,  2.04706799,  0.015     ],[-3.49512611,  2.71976847,  0.015     ],[-1.77823525,  2.62322206,  0.015     ],[ 0.13472833,  3.15590436,  0.015     ],[ 1.84582087,  3.15608509,  0.015     ],[ 3.14451749,  2.65597892,  0.015     ],])

    # generate coordinates to evaluate desired bearings
    x_pos = np.linspace(-3.0,3.0,N_x)
    y_pos = np.linspace(-3.0,3.0,N_y)
    z_pos = 1.0*np.ones(N)

    x_grid, y_grid = np.meshgrid(x_pos,y_pos)
    D = np.vstack((x_grid.flatten(),y_grid.flatten(),z_pos)).T

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

    webots = WebotsLauncher(world=os.path.join(package_dir, 'worlds', 'huge_formation.wbt'))

    launch_description.append(webots)

    # add executables for each robot
    for i in range(N):

        in_neighbors  = np.nonzero(Adj[:, i])[0].tolist()
        out_neighbors = np.nonzero(Adj[i, :])[0].tolist()
        init_pos = P[i, :].tolist()
        is_leader = True if i<2 else False
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
