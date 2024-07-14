import rclpy
from crazychoir.controller import VelocityController, VelocityCtrlStrategy
from crazychoir.planner import PositionTraj
from crazychoir.crazyflie_communication import VelSender

def main():
    rclpy.init()

    frequency = 10

    velocity_strategy = VelocityCtrlStrategy(frequency)
    sender = VelSender(frequency)
    desired_reference = PositionTraj()

    controller = VelocityController(pose_handler='pubsub', pose_topic='odom', pose_callback=None,
                velocity_strategy=velocity_strategy,
                command_sender=sender, traj_handler=desired_reference)

    controller.get_logger().info('Go!')

    rclpy.spin(controller)
    rclpy.shutdown()