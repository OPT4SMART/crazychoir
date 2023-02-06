import numpy
from rclpy.node import Node
from std_msgs.msg import Empty
from trajectory_msgs.msg import JointTrajectory as Trajectory, JointTrajectoryPoint as TrajectoryPoint
from .spline import DrawSpline

class ControlPanel(Node):

    def __init__(self):
        super().__init__('gui',
                            allow_undeclared_parameters=True, 
                            automatically_declare_parameters_from_overrides=True)

        self.n_agents = self.get_parameter('n_agents').value
        self.publishers_traj_params = {}

        for i in range(self.n_agents):
            topic = '/agent_{}/traj_params'.format(i)
            self.publishers_traj_params[i] = self.create_publisher(Trajectory, topic, 1)

        self.publisher_experiment  = self.create_publisher(Empty, '/experiment_trigger', 10)
        self.publisher_takeoff     = self.create_publisher(Empty, '/takeoff', 10)
        self.publisher_land        = self.create_publisher(Empty, '/land', 10)
        self.publisher_stop        = self.create_publisher(Empty, '/stop', 10)
        self.publisher_plot        = self.create_publisher(Empty, '/plot', 10)
        self.publisher_save_csv    = self.create_publisher(Empty, '/csv', 10)
        self.publisher_save_pickle = self.create_publisher(Empty, '/pickle', 10)
        

    def send_trajectory(self,id,t,x,y,z):

        msg = Trajectory()
        name = 'Agent_{}'.format(id)
        msg.joint_names.append(name)

        for i in range(len(t)):
            point = TrajectoryPoint()
            point.positions = [float(x[i]), float(y[i]), float(z)]
            point.velocities = [0.0,0.0,0.0]
            point.accelerations = [0.0,0.0,0.0]
            point.effort = [0.0,0.0,0.0]
            point.time_from_start.sec = int(t[i])
            point.time_from_start.nanosec = int((t[i]-int(t[i]))*1e9) #e9
            msg.points.append(point)
            # self.get_logger().info('Append point: ({}, {}, {}, {}) sent to cf{} from crazydraw'.format(point.time_from_start.nanosec,x[i],y[i],z,id))

        self.publishers_traj_params[id].publish(msg)

    
    def send_full_trajectory(self,file_name, id, time_scale, interpolation_scale, height = 1):
        self.get_logger().info('Send full trajectory command sent')
        x, y, t = DrawSpline.get_cords(file_name)
        t = [value * time_scale for value in t]
        x, y, t = DrawSpline.__distance_interpolation__(x, y, t, interpolation_scale)
        self.send_trajectory(id, t ,x, y, height)


    def goto(self,id,t,x,y,z):
        self.get_logger().info('Go-to command sent')
        height = z
        self.send_trajectory(id, [t] ,[x], [y], height)
 
    def takeoff(self):
        self.get_logger().info('Take-off command sent')
        msg = Empty()
        self.publisher_takeoff.publish(msg)

    def land(self):
        self.get_logger().info('Land command sent')
        msg = Empty()
        self.publisher_land.publish(msg)

    def start_experiment(self):
        self.get_logger().info('Start experiment command sent')
        msg = Empty()
        self.publisher_experiment.publish(msg)

    def stop(self):
        self.get_logger().info('Stop command sent')
        msg = Empty()
        self.publisher_stop.publish(msg)

    def plot(self):
        self.get_logger().info('Plot command sent')
        msg = Empty()
        self.publisher_plot.publish(msg)

    def save_csv(self):
        self.get_logger().info('Save .csv command sent')
        msg = Empty()
        self.publisher_save_csv.publish(msg)

    def save_pickle(self):
        self.get_logger().info('Save .pkl command sent')
        msg = Empty()
        self.publisher_save_pickle.publish(msg)

    def void(self):
        msg = Empty()
        self.publisher_takeoff.publish(msg)

    def notvoid(self):
        msg = Empty()
        self.publisher_land.publish(msg)