import rclpy

from choirbot import Pose
from choirbot.utils.position_getter import pose_subscribe


class RealPoseCtrl:
    def init(self, webots_node, properties):
        
        # Declare the robot name and fix the timestep
        self.robot = webots_node.robot
        self.timestep = int(self.robot.getBasicTimeStep())
        self.robot_name = self.robot.getName()
        self.gps = self.robot.getDevice("gps")
        self.gps.enable(self.timestep)

        # Initialize webots driver node
        rclpy.init(args=None)
        self.namespace = str(self.robot_name)
        self.intruder_id = int(self.robot_name[-1])
        self.intruder_driver = rclpy.create_node(
                            'real_pose_driver',
                            namespace=self.namespace,
                            allow_undeclared_parameters=True,
                            automatically_declare_parameters_from_overrides=True)

        self.robot.root_node_ref = self.robot.getSelf()
        self.robot.root_translation_field = self.robot.root_node_ref.getField("translation")
        self.robot.root_rotation_field = self.robot.root_node_ref.getField("rotation")
        
        self.real_pose = Pose(None, None, None, None)

    def step(self):
        rclpy.spin_once(self.intruder_driver, timeout_sec=0)

        if self.real_pose.position is None:
            return 
        
        position_list = list(self.real_pose.position)
        
        self.robot.root_translation_field.setSFVec3f(position_list)

class RealPoseCtrlLighthouse(RealPoseCtrl):
    def init(self, webots_node, properties):
        super().init(webots_node, properties)

        print('RealPoseCtrlLightHouse Started!')

        pose_handler = 'pubsub'
        pose_topic = f'/agent_{self.robot_name[-1]}/cf_odom'
        pose_callback = None
        
        self.subscription = pose_subscribe(pose_handler, pose_topic, self.intruder_driver, self.real_pose, pose_callback)


from crazychoir.filtering import Estimator

class RealPoseCtrlVicon(RealPoseCtrl):
    def init(self, webots_node, properties):
        super().init(webots_node, properties)

        print('RealPoseCtrlVicon Started!')

        pose_handler = 'vicon'
        pose_topic = f'/vicon/cf{self.robot_name[-1]}/cf{self.robot_name[-1]}'
        pose_callback = Estimator()
        
        self.subscription = pose_subscribe(pose_handler, pose_topic, self.intruder_driver, self.real_pose, pose_callback)
        pose_callback.set_pose(self.real_pose) 
     


