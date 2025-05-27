import cv2 as cv
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.parameter_service import SetParametersResult
from sensor_msgs.msg import Image


class Masker(Node):
    
    params_dict = {}
    
    
    def __init__(self):
        super().__init__("masker")
        
        self.declare_parameters(namespace="", parameters=[
            ("white_mask.hue_l",    0),
            ("white_mask.hue_h",    0),
            ("white_mask.sat_l",    0),
            ("white_mask.sat_h",    0),
            ("white_mask.val_l",    0),
            ("white_mask.val_h",    0),
            ("yellow_mask.hue_l",   0),
            ("yellow_mask.hue_h",   0),
            ("yellow_mask.sat_l",   0),
            ("yellow_mask.sat_h",   0),
            ("yellow_mask.val_l",   0),
            ("yellow_mask.val_h",   0),
            ("image_topic",         "image_raw"),
        ])
        
        self.params_dict["white_mask.hue_l"] = int(self.get_parameter("white_mask.hue_l").value)
        self.params_dict["white_mask.hue_h"] = int(self.get_parameter("white_mask.hue_h").value)
        self.params_dict["white_mask.sat_l"] = int(self.get_parameter("white_mask.sat_l").value)
        self.params_dict["white_mask.sat_h"] = int(self.get_parameter("white_mask.sat_h").value)
        self.params_dict["white_mask.val_l"] = int(self.get_parameter("white_mask.val_l").value)
        self.params_dict["white_mask.val_h"] = int(self.get_parameter("white_mask.val_h").value)
        self.params_dict["yellow_mask.hue_l"] = int(self.get_parameter("yellow_mask.hue_l").value)
        self.params_dict["yellow_mask.hue_h"] = int(self.get_parameter("yellow_mask.hue_h").value)
        self.params_dict["yellow_mask.sat_l"] = int(self.get_parameter("yellow_mask.sat_l").value)
        self.params_dict["yellow_mask.sat_h"] = int(self.get_parameter("yellow_mask.sat_h").value)
        self.params_dict["yellow_mask.val_l"] = int(self.get_parameter("yellow_mask.val_l").value)
        self.params_dict["yellow_mask.val_h"] = int(self.get_parameter("yellow_mask.val_h").value)
        
        # subscriptions
        self.create_subscription(Image, self.get_parameter("image_topic").value, self.image_cb, 1)
        self.add_on_set_parameters_callback(self.params_cb)
        
        # publishers
        self.white_mask_pub = self.create_publisher(Image, "white_mask", 1)
        self.yellow_mask_pub = self.create_publisher(Image, "yellow_mask", 1)
        self.mask_pub = self.create_publisher(Image, "mask", 1)
        
        self.bridge = CvBridge()
        
    def params_cb(self, params):
        for param in params:
            self.params_dict[param.name] = param.value
        # self.get_logger().log("Got new params", 20)
        return SetParametersResult(successful=True)
    
    def image_cb(self, ros_image):
        white_tcolLower = (self.params_dict["white_mask.hue_l"], self.params_dict["white_mask.sat_l"], self.params_dict["white_mask.val_l"])
        white_tcolUpper = (self.params_dict["white_mask.hue_h"], self.params_dict["white_mask.sat_h"], self.params_dict["white_mask.val_h"])
        yellow_tcolLower = (self.params_dict["yellow_mask.hue_l"], self.params_dict["yellow_mask.sat_l"], self.params_dict["yellow_mask.val_l"])
        yellow_tcolUpper = (self.params_dict["yellow_mask.hue_h"], self.params_dict["yellow_mask.sat_h"], self.params_dict["yellow_mask.val_h"])
        
        im = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        im = cv.medianBlur(im, 5)
        hsv_image = cv.cvtColor(im, cv.COLOR_BGR2HSV)
        white_mask = cv.inRange(hsv_image, white_tcolLower, white_tcolUpper)
        yellow_mask = cv.inRange(hsv_image, yellow_tcolLower, yellow_tcolUpper)
        yw_mask = cv.bitwise_or(white_mask, yellow_mask)
        
        self.white_mask_pub.publish(self.bridge.cv2_to_imgmsg(white_mask, "mono8"))
        self.yellow_mask_pub.publish(self.bridge.cv2_to_imgmsg(yellow_mask, "mono8"))
        self.mask_pub.publish(self.bridge.cv2_to_imgmsg(yw_mask, "mono8"))
        
        # self.blob_debug_pub.publish(self.bridge.cv2_to_imgmsg(mask, "mono8"))
        
        
def main(args=None):
    rclpy.init(args=args)
    node = Masker()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
        
if __name__ == '__main__':
    main()
    
        
        
        