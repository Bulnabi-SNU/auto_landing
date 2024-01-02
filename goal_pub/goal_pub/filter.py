import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from collections import deque
import numpy as np

class Filter(Node):

    def __init__(self):
        super().__init__('filter')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'bezier_waypoint/raw',
            self.tag_callback,
            10
        )
        self.publisher = self.create_publisher(Float32MultiArray, 'bezier_waypoint', 10)
        self.raw_values = deque([])

    def tag_callback(self, msg):
        tag_world = np.array(msg.data)
        if len(self.raw_values) < 15:
            self.raw_values.append(tag_world)
        else:
            self.raw_values.popleft()
            self.raw_values.append(tag_world)
        average = sum(self.raw_values)/len(self.raw_values)
        avg_msg = Float32MultiArray()
        avg_msg.data = average.tolist()
        
        self.publisher.publish(avg_msg)

def main(args=None):
    rclpy.init(args=args)
    filter = Filter()
    rclpy.spin(filter)

    sub.destroy_node()
    rclpy.shutdown

if __name__ == '__main__':
    main()
