import rclpy
from crazychoir.integrator import CrazyflieIntegrator


def main():
    rclpy.init()

    frequency = 100

    integrator = CrazyflieIntegrator(frequency)

    rclpy.spin(integrator)
    rclpy.shutdown()
