from .radio_handler_vel import RadioHandlerVel
import time
from cflib.crazyflie.log import LogConfig
from nav_msgs.msg import Odometry

class RadioHandlerVelLightHouse(RadioHandlerVel):

    def __init__(self):
        super().__init__()
        print('Radio Vel Lighthouse Started!')

    def set_log(self, uri, cf, log_period_in_ms, agent_id):
        """
        Logging groups and variables retrived from 
            https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/api/logs/
        """

        # The definition of the logconfig can be made before connecting
        self._lg_stab = LogConfig(name='Stabilizer', period_in_ms=log_period_in_ms)
        self._lg_stab.add_variable('kalman.stateX', 'float')
        self._lg_stab.add_variable('kalman.stateY', 'float')
        self._lg_stab.add_variable('kalman.stateZ', 'float')
        
        # The fetch-as argument can be set to FP16 to save space in the log packet
        # self._lg_stab.add_variable('pm.vbat', 'FP16')

        
        try:
            cf.log.add_config(self._lg_stab)
            # This callback will receive the data
            self._lg_stab.data_received_cb.add_callback(lambda timestamp, data, logconf: self._stab_log_data(timestamp, data, logconf, agent_id))
            # This callback will be called on errors
            self._lg_stab.error_cb.add_callback(self._stab_log_error)
            # Start the logging
            self._lg_stab.start()
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Stabilizer log config, bad configuration.')


    def _stab_log_data(self, timestamp, data, logconf, agent_id):
            odom = Odometry()
            odom.pose.pose.position.x = data['kalman.stateX']
            odom.pose.pose.position.y = data['kalman.stateY']
            odom.pose.pose.position.z = data['kalman.stateZ']
            odom.twist.twist.linear.x = 0.0
            odom.twist.twist.linear.y = 0.0
            odom.twist.twist.linear.z = 0.0
            odom.pose.pose.orientation.x = 0.0
            odom.pose.pose.orientation.y = 0.0
            odom.pose.pose.orientation.z = 0.0
            odom.pose.pose.orientation.w = 1.0
            odom.twist.twist.angular.x = 0.0
            odom.twist.twist.angular.y = 0.0
            odom.twist.twist.angular.z = 0.0
            self.pub_odom[agent_id].publish(odom)
