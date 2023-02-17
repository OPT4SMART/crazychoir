import rclpy
from rclpy.node import Node
from crazychoir.planner import PointToPointPlanner_2D
from rclpy.executors import MultiThreadedExecutor


def main():
    rclpy.init()

    planner_params = Node('planner_params', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
    vicon_id = planner_params.get_parameter('vicon_id').value
    
    goal_tolerance = 0.05

    planner = PointToPointPlanner_2D(goal_tolerance, pose_handler='vicon', pose_topic='/vicon/cf{}/cf{}'.format(vicon_id,vicon_id))
    
    executor = MultiThreadedExecutor()
    rclpy.spin(planner, executor)
    rclpy.shutdown()
