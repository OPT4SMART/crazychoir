import rclpy
from rclpy.node import Node
from choirbot.guidance.task import TaskGuidance, PositionTaskExecutor
from choirbot.optimizer import TaskOptimizer

def main():
    rclpy.init()

    guidance_params = Node('guidance_params', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
    vicon_id = guidance_params.get_parameter('vicon_id').value

    # initialize task guidance
    opt_settings = {'max_iterations': 20}
    executor = PositionTaskExecutor()
    optimizer = TaskOptimizer(settings=opt_settings)
    
    guidance = TaskGuidance(optimizer=optimizer, executor=executor, data=None, pose_handler='vicon', pose_topic='/vicon/cf{}/cf{}'.format(vicon_id,vicon_id))

    rclpy.spin(guidance)
    rclpy.shutdown()
