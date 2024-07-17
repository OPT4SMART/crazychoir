import rclpy
from choirbot.collision import SingleIntegratorCollisionAvoidance
from choirbot.collision.safety_filters import SingleIntegratorCBF

from geometry_msgs.msg import Twist
import numpy as np

from rclpy.node import Node
def main():
    rclpy.init()


    collision_params = Node('collision_params', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
    freq = collision_params.get_parameter('freq').value

    node_frequency = freq

    safe_distance = 0.3       # m
    gamma = 1.0               # safety parameter
    max_velocity = 0.1 # [m/s] NOTE: These value is the maximum velocity for the single integrator dynamic.
    max_braking_velocity = 0.1 # [m/s] NOTE: These value is the maximum braking velocity for the single integrator dynamic.
    
    safety_filter = SingleIntegratorCBF(
        distance = safe_distance, 
        gamma = gamma, 
        max_velocity = max_velocity,
        max_braking_velocity = max_braking_velocity
        )


    collision_avoidance = SingleIntegratorCollisionAvoidance(
        pose_handler='pubsub', 
        pose_topic='odom', 
        pose_callback=None, 
        node_frequency=node_frequency, 
        topic_msg=Twist,
        topic_name='cmd_vel', 
        safety_filter=safety_filter
        )

    rclpy.spin(collision_avoidance)
    rclpy.shutdown()
