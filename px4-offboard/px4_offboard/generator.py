import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray


class Generator(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/bezier_waypoint', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Float32MultiArray()
        msg.data = [0., 0., -10., 0., 0., -0.5] # in order of xf and vf
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_generator = Generator()

    rclpy.spin(minimal_generator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_generator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
