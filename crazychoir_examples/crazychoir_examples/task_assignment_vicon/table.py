import rclpy
from rclpy.node import Node
from choirbot.guidance.task import PositionTaskTable
from std_msgs.msg import Empty

class TriggerNode(Node):
    def __init__(self):
        super().__init__('table_parameters', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.n_agents = self.get_parameter('N').value
        self.create_subscription(Empty, '/experiment_trigger', self.experiment_trigger, 10)

    def experiment_trigger(self, _):
        self.get_logger().info('Starting experiment')
        
        table = PositionTaskTable(self.n_agents)
        table.gc.trigger()

        rclpy.spin(table)
        rclpy.shutdown()

def main():
    rclpy.init()
    trigger = TriggerNode()
    rclpy.spin_once(trigger)
    # rclpy.spin(trigger)
    rclpy.shutdown()

if __name__ == '__main__':
    main()