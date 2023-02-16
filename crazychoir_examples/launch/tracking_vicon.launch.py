from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

node_frequency = 100 # [Hz]

def generate_launch_description():

    # number of agents
    N = 1

    # Set uri address 
    uri = 'radio://0/80/2M/E7E7E7E701'

    initial_position = [0.0, 0.0, 0.015]

    # initialize launch description
    launch_description = []
    radio_launch = []

    # Launch control Panel
    launch_description.append(Node(
                package='crazychoir_examples', 
                executable='crazychoir_tracking_vicon_gui',
                output='screen',
                parameters=[{
                    'n_agents': 1,
                    }]))
            
    # Launch radio node
    radio_id = int(uri.split('/')[2])
    radio_launch.append(Node(
        package='crazychoir_examples', 
        executable='crazychoir_tracking_vicon_radio', 
        output='screen',
        prefix='xterm -title "radio_{}" -hold -e'.format(radio_id),
        namespace='radio_{}'.format(radio_id),
        parameters=[{
            'uris': [uri],
            'agent_ids': [0],
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

    # controller
    launch_description.append(Node(
        package='crazychoir_examples', 
        executable='crazychoir_tracking_vicon_controller', 
        namespace='agent_0',
        output='screen',
        parameters=[{
            'agent_id': 0,
            }]))

    # guidance
    launch_description.append(Node(
        package='crazychoir_examples', 
        executable='crazychoir_tracking_vicon_guidance', 
        namespace='agent_0',
        output='screen',
        parameters=[{
            'agent_id': 0,
            'init_pos': initial_position,
            }]))


    # reference
    launch_description.append(Node(
        package='crazychoir_examples',
        executable='crazychoir_tracking_vicon_trajectory', 
        namespace='agent_0',
        output='screen',
        parameters=[{     
            'agent_id': 0, 
            }]))

    # include delayed radio launcher
    timer_action = TimerAction(period=5.0, actions=[LaunchDescription(radio_launch)])
    launch_description.append(timer_action)

    return LaunchDescription(launch_description)
