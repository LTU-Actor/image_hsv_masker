import cv2 as cv
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.parameter_service import SetParametersResult
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class Masker(Node):
    
    params_dict = {}
    
    
    def __init__(self):
        super().__init__("masker")
        
        # declare HSV mask params
        self.declare_parameter("hue_l", 0)
        self.declare_parameter("hue_h", 30)
        self.declare_parameter("sat_l", 0)
        self.declare_parameter("sat_h", 255)
        self.declare_parameter("val_l", 150)
        self.declare_parameter("val_h", 255)
        self.params_dict["hue_l"] = int(self.get_parameter("hue_l").value)
        self.params_dict["hue_h"] = int(self.get_parameter("hue_h").value)
        self.params_dict["sat_l"] = int(self.get_parameter("sat_l").value)
        self.params_dict["sat_h"] = int(self.get_parameter("sat_h").value)
        self.params_dict["val_l"] = int(self.get_parameter("val_l").value)
        self.params_dict["val_h"] = int(self.get_parameter("val_h").value)
        
        self.declare_parameter("image_topic", "image_raw")
        
        # subscriptions
        self.create_subscription(Image, self.get_parameter("image_topic").value, self.image_cb, 1)
        self.add_on_set_parameters_callback(self.params_cb)
        
        # publishers
        self.blob_debug_pub = self.create_publisher(Image, "mask", 1)
        
        self.bridge = CvBridge()
        
    def params_cb(self, params):
        for param in params:
            self.params_dict[param.name] = param.value
        # self.get_logger().log("Got new params", 20)
        return SetParametersResult(successful=True)
    
    def image_cb(self, ros_image):
        im = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        im = cv.medianBlur(im, 5)
        hsv_image = cv.cvtColor(im, cv.COLOR_BGR2HSV)
        tcolLower = (self.params_dict["hue_l"], self.params_dict["sat_l"], self.params_dict["val_l"])
        tcolUpper = (self.params_dict["hue_h"], self.params_dict["sat_h"], self.params_dict["val_h"])
        mask = cv.inRange(hsv_image, tcolLower, tcolUpper)
        
        self.blob_debug_pub.publish(self.bridge.cv2_to_imgmsg(mask, "mono8"))
        
        
def main(args=None):
    rclpy.init(args=args)
    node = Masker()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
        
if __name__ == '__main__':
    main()
    
        
        
        