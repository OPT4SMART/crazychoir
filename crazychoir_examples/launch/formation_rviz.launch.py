from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import numpy as np
import os


def generate_launch_description():

    # set rng seed
    np.random.seed(1)

    # number of agents
    N = 4

    # communication matrix
    Adj = np.array([
        [0, 1, 0, 1],
        [1, 0, 1, 1],
        [0, 1, 0, 1],
        [1, 1, 1, 0],
    ])

    # generate initial positions
    P = np.zeros((N, 3))
    # fix static leader positions
    P[0,0:3] = np.array([1.0, 0.0, 1.0])
    P[1,0:3] = np.array([2.0, 0.0, 1.0])
    P[2,0:3] = np.array([0.0, 0.0, 1.0])
    P[3,0:3] = np.array([0.0, 0.0, 1.0])
    # set random follower positions on x-y plane
    P[2:4,0:2] = 2.0*np.random.rand(2,2)

    # compute orthogonal projection matrix
    # generate coordinates of a square to evaluate desired bearings
    D = np.zeros((N, 3))
    D[0,0:3] = np.array([-1, -1, 1])
    D[1,0:3] = np.array([ 1, -1, 1])
    D[2,0:3] = np.array([ 1,  1, 1])
    D[3,0:3] = np.array([-1,  1, 1])
    dd = np.size(D,1)

    orth_proj = np.zeros((dd*N, dd*N))
    for i in range(N):
        neigh_ii = np.nonzero(Adj[i, :])[0].tolist()
        for j in neigh_ii:
            x_des_i = D[i, :]
            x_des_j = D[j, :]
            # compute desired bearing
            g_ij = np.matrix((x_des_j - x_des_i)/np.linalg.norm(x_des_j - x_des_i))
            # fill the matrix
            orth_proj[i*dd:i*dd+N-1,j*dd:j*dd+N-1] = np.eye(dd) - g_ij.T @ g_ij

    # initialize launch description with rviz executable
    rviz_config_dir = get_package_share_directory('crazychoir_examples')
    rviz_config_file = os.path.join(rviz_config_dir, 'rvizconf.rviz')
    launch_description = [Node(package='rviz2', executable='rviz2', output='screen',
    arguments=['-d', rviz_config_file])]
    
    # add executables for each robot
    for i in range(N):

        in_neighbors  = np.nonzero(Adj[:, i])[0].tolist()
        out_neighbors = np.nonzero(Adj[i, :])[0].tolist()
        position = P[i, :].tolist()
        is_leader = True if i < 2 else False
        orth_proj_array = orth_proj[i*dd:(i+1)*dd,:].flatten().tolist()

        # guidance
        launch_description.append(Node(
            package='crazychoir_examples',
            executable='crazychoir_formation_rviz_guidance', 
            output='screen',
            namespace='agent_{}'.format(i),
            parameters=[{'agent_id': i, 'N': N, 'dd': dd, 'in_neigh': in_neighbors, 'out_neigh': out_neighbors, 'is_leader': is_leader, 'ort_proj': orth_proj_array}]))

        # controller
        launch_description.append(Node(
            package='crazychoir_examples', 
            executable='crazychoir_formation_rviz_controller', 
            output='screen',
            namespace='agent_{}'.format(i),
            parameters=[{'agent_id': i}]))

        # integrator
        launch_description.append(Node(
            package='crazychoir_examples', 
            executable='crazychoir_formation_rviz_integrator', 
            output='screen',
            namespace='agent_{}'.format(i),
            parameters=[{'agent_id': i, 'init_pos': position}]))

        # RVIZ visualization
        launch_description.append(Node(
            package='crazychoir_examples', 
            executable='crazychoir_formation_rviz_rviz', 
            output='screen',
            namespace='agent_{}'.format(i),
            parameters=[{'agent_id': i}]))

    return LaunchDescription(launch_description)
