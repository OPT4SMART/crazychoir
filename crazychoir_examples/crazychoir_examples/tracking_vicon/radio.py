import rclpy
from crazychoir.radio_handler import RadioHandlerFPQR

def main():
    rclpy.init()
    
    radio = RadioHandlerFPQR()

    rclpy.spin(radio)
    rclpy.shutdown()
