import rclpy
from choirbot.collision import ClosestRobotGetter


def main():
    rclpy.init()

    sensing_distance = 2.0
    closest_robot_getter = ClosestRobotGetter(sensing_distance=sensing_distance)

    rclpy.spin(closest_robot_getter)
    rclpy.shutdown()
