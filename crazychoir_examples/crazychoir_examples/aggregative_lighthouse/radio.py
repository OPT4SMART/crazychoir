import rclpy
from rclpy.node import Node
from crazychoir.radio_handler import RadioHandlerVelLightHouse


def main():
    rclpy.init()

    radio = RadioHandlerVelLightHouse()

    rclpy.spin(radio)
    rclpy.shutdown()
