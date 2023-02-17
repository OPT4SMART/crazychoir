import rclpy
from rclpy.node import Node
from crazychoir.radio_handler import RadioHandlerXYZ


def main():
    rclpy.init()

    radio = RadioHandlerXYZ()

    rclpy.spin(radio)
    rclpy.shutdown()
