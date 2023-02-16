import rclpy
from crazychoir.planner import PointToPointPlanner_2D
from rclpy.executors import MultiThreadedExecutor


def main():
    rclpy.init()

    goal_tolerance = 0.05

    planner = PointToPointPlanner_2D(goal_tolerance, pose_handler='pubsub', pose_topic='odom')
    
    executor = MultiThreadedExecutor()
    rclpy.spin(planner, executor)
    rclpy.shutdown()
