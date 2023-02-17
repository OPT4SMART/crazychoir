import rclpy
from rclpy.node import Node
from crazychoir.radio_handler import RadioHandlerFPQR, RadioHandlerFRPY


def main():
    rclpy.init()

    radio = RadioHandlerFPQR()

    rclpy.spin(radio)
    rclpy.shutdown()
