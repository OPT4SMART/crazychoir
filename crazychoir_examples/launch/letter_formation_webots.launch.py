import os
import numpy as np
import pathlib

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from webots_ros2_driver.webots_launcher import WebotsLauncher

node_frequency = 100 # [Hz]


def get_webots_driver_cf(agent_id):
    package_dir_driver = get_package_share_directory('crazychoir_examples')
    robot_description = pathlib.Path(os.path.join(package_dir_driver, 'crazyflie_fpqr.urdf')).read_text()
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
    P[0]  = np.array([ 0.550,  0.874,  0.015])
    P[1]  = np.array([-0.708, -0.655,  0.015])
    P[2]  = np.array([ 0.392,  0.396,  0.015])
    P[3]  = np.array([ 0.325, -0.626,  0.015])
    P[4]  = np.array([ 0.051, -0.725,  0.015])
    P[5]  = np.array([-0.103, -0.942,  0.015])
    P[6]  = np.array([-0.017, -1.221,  0.015])
    P[7]  = np.array([-0.323,  0.422,  0.015])
    P[8]  = np.array([-0.975,  0.725,  0.015])
    P[9]  = np.array([-0.407,  0.913,  0.015])
    P[10] = np.array([-0.049,  1.191,  0.015])

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

    # Generate Webots world
    robots = [{
                'name': f'agent_{i}',
                'type': 'crazyflie_red', 
                'position': P[i, :].tolist(),
            } for i in range(2)]
    robots += [{
                'name': f'agent_{i}',
                'type': 'crazyflie', 
                'position': P[i, :].tolist(),
            } for i in range(2,N)]
    
    world_package_dir = get_package_share_directory('crazychoir_examples')
    source_filename = os.path.join(world_package_dir, 'worlds', 'empty_world.wbt')
    target_filename = os.path.join(world_package_dir, 'worlds', 'my_world.wbt')
    generate_webots_world_file(robots, source_filename, target_filename)            
    webots = WebotsLauncher(world=target_filename)
    launch_description.append(webots)

    # Launch control Panel
    launch_description.append(Node(
                package='crazychoir_examples', 
                executable='crazychoir_formation_webots_gui',
                output='screen',
                parameters=[{
                    'n_agents': N,
                    }]))

            
    # add executables for each robot
    for i in range(N):

        in_neighbors  = np.nonzero(Adj[:, i])[0].tolist()
        out_neighbors = np.nonzero(Adj[i, :])[0].tolist()
        init_pos = P[i, :].tolist()
        is_leader = True if i < 2 else False
        orth_proj_array = orth_proj[i*dd:(i+1)*dd,:].flatten().tolist()

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
