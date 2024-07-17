import rclpy
from crazychoir.guidance.aggregative import AggregativeGuidanceCrazyflie, AggregativeGuidanceCrazyflieMovingIntruder
from choirbot.optimizer import AggregativeOptimizer
from rclpy.node import Node

from geometry_msgs.msg import Vector3

def main():
    rclpy.init()

    node = Node('guid_cf', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
    
    max_iter = node.get_parameter('max_iter').value

    opt_settings = {
        'agent_dim': 2,             # dimension of agent's optimization variable
        'stepsize':5e-3,            # stepsize for gradient tracking
        #
        'gamma_intruder': 1.0,      # weight for ||x_i - r_i||^2
        'gamma_target': 1.0,        # weight for ||\sigma(x) - b_i||^2
        'gamma_shape': 1.0,         # weight for ||x_i - \sigma(x)||^2
        #
        'max_iterations': max_iter,
        'enable_log': False,
        }

    optimizer = AggregativeOptimizer(settings=opt_settings)

    freq_reference = node.get_parameter('freq_reference').value
    agent_id = node.get_parameter('agent_id').value
    # height = 0.5 + agent_id*0.15
    height = 1.0
    guidance = AggregativeGuidanceCrazyflieMovingIntruder(optimizer, height, freq_reference, 'pubsub', 'odom', None,  'position', Vector3)

    rclpy.spin(guidance)
    rclpy.shutdown()
