import numpy as np
import pathlib
import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from disropt.utils.graph_constructor import binomial_random_graph, ring_graph, metropolis_hastings
from launch.actions import TimerAction


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
        prefix=f'xterm  -title "driver_{agent_id}" -geometry 160x7+1980 -hold -e ',# if i == 0 else None,
        output='screen',
        additional_env={
            'WEBOTS_ROBOT_NAME':f'agent_{agent_id}',
            },
        parameters=[
            {'robot_description': robot_description},
        ]
    )

    return crazyflie_driver

def get_real_pose_webots_driver(name):
    package_dir_driver = get_package_share_directory('crazychoir_examples')
    robot_description = pathlib.Path(os.path.join(package_dir_driver, 'real_pose_lighthouse.urdf')).read_text()
    crazyflie_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        namespace=f'{name}',
        output='screen',
        prefix=f'xterm  -title "driver_{name}" -geometry 160x7+1980 -hold -e ',# if i == 0 else None,
        additional_env={
            'WEBOTS_ROBOT_NAME':f'{name}',
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

    # Set uri address for each quadrotor
    uris = [
        'radio://0/100/2M/E7E7E7E700',
        'radio://0/100/2M/E7E7E7E701',
        'radio://0/100/2M/E7E7E7E702'
    ]

    N = 3
    max_iter = 20000


    # Generate communication graph
    Adj = binomial_random_graph(N, p=0.5, seed=3)
    W = metropolis_hastings(Adj)

    P = np.zeros((2*N, 3))
    P[0] = np.array([ 0.5, -0.2, 0.0])
    P[1] = np.array([-0.3, -0.5, 0.0])
    P[2] = np.array([-0.1,  0.5, 0.0])
    P[3] = np.array([-1.0, -1.0, 0.0])
    P[4] = np.array([-1.0,  1.0, 0.0])
    P[5] = np.array([ 1.0,  1.0, 0.0])

    # Project real crazyflie movements in the simulation
    robots =[{
            'name': f'real_cf_{i}',
            'type': 'real_pose', 
            'position': P[i, :].tolist(), 
        } for i in range(N)]


    # Target
    target = [0.0, 0.0, 1.0]
    robots +=[{
                'name': 'target',
                'type': 'target', 
                'position': target, 
            }]

    # Simulated Intruders
    robots += [{
                'name': f'agent_{i}',
                'type': 'crazyflie_red', 
                'position': P[i, :].tolist(), 
            } for i in range(N,2*N) ]
    
    intruders_goals = {
    # id: [[t, x, y, z], ...]
        3: [[10,-1.0, -1.0, 1.0], [30,-0.9,  0.9, 1.0], [30, 0.75,  0.75, 1.0]],
        4: [[10,-1.0,  1.0, 1.0], [30, 0.9,  0.9, 1.0], [30, 0.75, -0.75, 1.0]],
        5: [[10, 1.0,  1.0, 1.0], [30, 0.9, -0.9, 1.0], [30,-0.75, -0.75, 1.0]],
    }

    launch_description = []
    radio_launch = []

    # Generate Webots World
    world_package_dir = get_package_share_directory('crazychoir_examples')
    source_filename = os.path.join(world_package_dir, 'worlds', 'empty_world.wbt')
    target_filename = os.path.join(world_package_dir, 'worlds', 'my_world.wbt')
    generate_webots_world_file(robots, source_filename, target_filename)            
    webots = WebotsLauncher(world=target_filename)
    launch_description.append(webots)

    # Launch control Panel
    launch_description.append(Node(
                package='crazychoir_examples', 
                executable='crazychoir_aggregative_lighthouse_gui',
                output='screen',
                parameters=[{
                    'n_agents': 2*N,
                    }]))
    
    # Launch radio node
    radios = set([int(uri.split('/')[2]) for uri in uris])
    for r in radios:
        uris_r = [uri for uri in uris if int(uri.split('/')[2]) == r]
        agent_ids_r = [uris.index(uri_r) for uri_r in uris_r]
        radio_launch.append(Node(
            package='crazychoir_examples', 
            executable='crazychoir_aggregative_lighthouse_radio', 
            prefix='xterm -title "radio_{}" -hold -e'.format(r),
            namespace='radio_{}'.format(r),
            parameters=[{
                'uris': uris_r,
                'agent_ids': agent_ids_r,
                'cmd_topic': 'cmd_vel',
                'logger' : True,
                }]))  

    # add executables for each robot
    for i in range(2*N):
        initial_position = P[i, :].tolist()
    

        if i < N:
            launch_description.append(get_real_pose_webots_driver(f'real_cf_{i}'))
            launch_description.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            additional_env={'WEBOTS_ROBOT_NAME':f'real_cf_{i}'},
            namespace=f'real_cf_{i}',
            output='screen',
            parameters=[{
                'robot_description': '<robot name=""><link name=""/></robot>',
                }]))
        
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
            
            # simple guidance (takeoff and landing)
            launch_description.append(Node(
                package='crazychoir_examples', 
                executable='crazychoir_aggregative_lighthouse_simple_guidance', 
                output='screen',
                namespace=f'agent_{i}',
                parameters=[{
                    'agent_id': i, 
                    'init_pos': initial_position,
                    }]))

            # controller
            launch_description.append(Node(
                package='crazychoir_examples', 
                executable='crazychoir_aggregative_lighthouse_controller', 
                namespace=f'agent_{i}',
                output='screen',
                prefix=f'xterm  -title "controller_{i}" -geometry 160x4+1980+{int(95*(i+N))} -hold -e ',# if i == 0 else None,
                parameters=[{
                    'freq': freq_controller,                       
                    'agent_id': i,
                    }]))
            
            # trajectory
            launch_description.append(Node(
                package='crazychoir_examples',
                executable='crazychoir_aggregative_lighthouse_trajectory', 
                namespace=f'agent_{i}',
                output='screen',
                parameters=[{     
                    'freq': freq_reference,                       
                    'agent_id': i, 
                    }]))
        
        else:
            # Intruders
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
        
    # include delayed radio launcher
    timer_action = TimerAction(period=1.0, actions=[LaunchDescription(radio_launch)])
    launch_description.append(timer_action)

    return LaunchDescription(launch_description)
