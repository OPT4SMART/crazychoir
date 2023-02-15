from typing import Callable

from rclpy.node import Node
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist 
from trajectory_msgs.msg import JointTrajectory as Trajectory, JointTrajectoryPoint as TrajectoryPoint

from choirbot import Pose
from choirbot.utils.position_getter import pose_subscribe


import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import Swarm


class RadioHandler(Node):

    def __init__(self):
        super().__init__('radio', allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True)
        
        print('Radio Started!')

        # Get Launcher Parameters
        self.uris = self.get_parameter('uris').value
        self.agent_ids = self.get_parameter('agent_ids').value
        self.cmd_topic = self.get_parameter('cmd_topic').value

        print("Uris associated to this radio   :\t{}".format(self.uris))
        print("Agents associated to this radio :\t{}\n".format(self.agent_ids))

        # Initialize subscriptions
        self.stop_subscription = self.create_subscription(Empty, '/stop', self.cmd_stop, 10)
        self.take_off_subscription = self.create_subscription(Empty, '/takeoff', self.cmd_take_off, 10)

        # Initialize Flags
        self.emergency_stop = False

        # Initialize radio drivers
        cflib.crtp.init_drivers()

        # Declare swarm instance
        self.factory = CachedCfFactory(rw_cache='./cache')
        self.swarm = Swarm(self.uris, factory=self.factory)  

        # Open communication links      
        self.swarm.open_links()

        for uri, scf in self.swarm._cfs.items():
            cf = scf.cf
            self.set_values(uri, cf)
            # self.read_values(uri, cf)
            # self.set_log(uri,cf)
            
        # Unlock startup thrust protection
        for uri, scf in self.swarm._cfs.items():
            cf = scf.cf
            cf.commander.send_setpoint(0,0,0,0)
            # print("[{}] \t  cf{} state = {}".format(time.time(), int(uri[-1]), scf.cf.state))
            self.get_logger().info('cf{} ready!'.format(int(uri[-1])))        
        

        # Initialize subscriptions to command topic for each crazyflie
        self.subscriptions_pose_list = {}
        self.subscriptions_cmd_list = {}
        self.current_poses = {}

        for i, value in enumerate(self.swarm._cfs.items()):
            uri = value[0]
            scf = value[1]
            cf = scf.cf

            # Poses Subscription
            # TODO: Verify that Kalman Filter works also with fpqr commands
            if self.cmd_topic == 'traj_params': 
                self.current_poses = {uri[-1]: Pose(None, None,None, None)}
                vicon_id = self.uris[i][-1]
                pose_handler = 'vicon'
                pose_topic = '/vicon/cf{}/cf{}'.format(vicon_id,vicon_id)
                pose_callback = SendViconPose()
                self.subscriptions_pose_list[uri[-1]] = pose_subscribe(pose_handler, pose_topic, self, self.current_poses[uri[-1]], pose_callback)
                pose_callback.set_callback(self.current_poses[uri[-1]],cf)

            # Commands Subscription
            agent_id = self.agent_ids[i]
            if self.cmd_topic == 'traj_params':
                msg_type = Trajectory
            elif self.cmd_topic == 'cmd_vel':
                msg_type = Twist
            else:
                msg_type = Empty
            topic_name = '/agent_{}/{}'.format(agent_id,self.cmd_topic)
            self.subscriptions_cmd_list[uri[-1]] = self.create_subscription(msg_type, topic_name, lambda msg, uri_i=uri, cf_i = cf: self.cmd_sender(msg, uri_i, cf_i), 10)

    def cmd_sender(self, msg, uri, cf):
        raise NotImplementedError

    def cmd_stop(self, _):
        raise NotImplementedError
    
    def cmd_take_off(self, _):
        raise NotImplementedError

    def set_values(self,uri, cf):
        raise NotImplementedError

    ################################
    # Logging methods
    ################################

    def set_log(self, uri, cf):

        # The definition of the logconfig can be made before connecting
        self._lg_stab = LogConfig(name='Stabilizer', period_in_ms=100)
        # self._lg_stab.add_variable('stateEstimate.x', 'float')
        # self._lg_stab.add_variable('stateEstimate.y', 'float')
        # self._lg_stab.add_variable('stateEstimate.z', 'float')
        # self._lg_stab.add_variable('stabilizer.roll', 'float')
        # self._lg_stab.add_variable('stabilizer.pitch', 'float')
        self._lg_stab.add_variable('stabilizer.yaw', 'float')
        # The fetch-as argument can be set to FP16 to save space in the log packet
        self._lg_stab.add_variable('pm.vbat', 'FP16')

        # Adding the configuration cannot be done until a Crazyflie is
        # connected, since we need to check that the variables we
        # would like to log are in the TOC.
        
        try:
            cf.log.add_config(self._lg_stab)
            # This callback will receive the data
            self._lg_stab.data_received_cb.add_callback(self._stab_log_data)
            # This callback will be called on errors
            self._lg_stab.error_cb.add_callback(self._stab_log_error)
            # Start the logging
            self._lg_stab.start()
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Stabilizer log config, bad configuration.')
    

    def _stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _stab_log_data(self, timestamp, data, logconf):
        """Callback from a the log API when data arrives"""
        print(f'[{timestamp}][{logconf.name}]: ', end='')
        for name, value in data.items():
            print(f'{name}: {value:3.3f} ', end='')
        print()

class SendViconPose(Callable):

    def __init__(self):
        self.count = 0
        self.past_time = 0.0
        pass

    def set_callback(self, current_pose, cf):
        self.current_pose = current_pose
        self.cf = cf

    def __call__(self):
        x  = self.current_pose.position[0]
        y  = self.current_pose.position[1]
        z  = self.current_pose.position[2]
        qx = self.current_pose.orientation[0]
        qy = self.current_pose.orientation[1]
        qz = self.current_pose.orientation[2]
        qw = self.current_pose.orientation[3]

        # TODO: Verify
        # self.cf.extpos.send_extpose(x, y, z, qx/qw, qy/qw, qz/qw, qw/qw)
        self.count += 1
        if not (self.count % 7): # Send ext pose at 10Hz
            self.cf.extpos.send_extpose(x, y, z, qx, qy, qz, qw)
            # print("[{:.2f}] \t  - cf{} external pose sent!".format(1/(time.time() - self.past_time), self.cf.link_uri[-1]))  
            # self.past_time = time.time()

