import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as ImgMsg
from bboxes_ex_msgs.msg import BoundingBoxes as BboxMsg
from zed_interfaces.msg import DepthInfoStamped as DepthMsg
from cv_bridge import CvBridge
import pyzed.sl as sl

class TagPublisher(Node):
    def __init__(self):
        super().__init__('tag_pose')        
        
        self.img_subscription = self.create_subscription(
            ImgMsg,
            'zed2i/zed_node/depth/depth_registered',
            self.img_callback,
            10
        )
        
        self.bbox_subscription = self.create_subscription(
            BboxMsg,
            'yolov5/bounding_boxes',
            self.bbox_callback,
            10
        )

        self.depth_subscription = self.create_subscription(
            DepthMsg,
            'zed2i/zed_node/depth/depth_info',
            self.depth_callback,
            10
        )
               
        print("Hmmmmmmm")
        
        
    def bbox_callback(self, msg):
        try:
            print("bbox callback")
            bbox = msg.bounding_boxes[0]
            self.prob = bbox.probability
            # self.get_logger().info("%s" % self.prob)
            self.bbox = [bbox.xmin, bbox.ymin, bbox.xmax, bbox.ymax]
        except:
            self.get_logger().info("Hey hey~ cross not detected")

    def img_callback(self, msg):
        try:
            print("img callback")
            # self.get_logger().info("%s" % self.prob)
            # if self.prob > 0.8:
            #     image = sl.Mat()
            # depth_map = sl.Mat()
            # runtime_parameters = sl.RuntimeParameters()
            # zed = sl.Camera()
            # if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS :
            #     # A new image and depth is available if grab() returns SUCCESS
            #     # zed.retrieve_image(image, sl.VIEW.LEFT) # Retrieve left image
            #     zed.retrieve_measure(depth_map, sl.MEASURE.DEPTH) # Retrieve depth
            #     depth = depth_map[self.bbox[0],self.bbox[1]]
            #     print(depth)
            bridge = CvBridge()
            img = bridge.imgmsg_to_cv2(msg,desired_encoding='passthrough')
            # minP = img[self.bbox[0],self.bbox[1]]
            # maxP = img[self.bbox[2],self.bbox[3]]
            # minD = self.min_d + self.depthPerPixel * minP
            # maxD = self.min_d + self.depthPerPixel * maxP
            # print(minD)

        except:
            self.get_logger().info("Oh no~ zed2i image not detected")

    def depth_callback(self, msg):
        try:
            print("info callback")
            self.min_d = msg.min_depth
            self.max_d = msg.max_depth
            self.depthPerPixel = (self.max_d - self.min_d) / 255
        except:
            self.get_logger().info("Ummm,,,, depth info not received")

def main(args=None):
    rclpy.init(args=args)
    tagpublisher = TagPublisher()
    rclpy.spin(tagpublisher)

    sub.destroy_node()
    rclpy.shutdown


if __name__ == '__main__':
    main()
