#!/usr/bin/python3
import cv2
import numpy as np
import yaml
from cv_bridge import CvBridge
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image

class Read_file():
    def __init__(self) -> None:
        pgm_file = cv2.imread('../maps/scenario1-2/map.pgm', 0)
        image = rospy.get_param("/custom_prefix/image")
        self.resolution = rospy.get_param("/custom_prefix/resolution")
        self.free_thresh = rospy.get_param("/custom_prefix/free_thresh")
        self.occupied_thresh = rospy.get_param("/custom_prefix/occupied_thresh")
        origin = rospy.get_param("/custom_prefix/origin")
        self.origin_x = origin[0]
        self.origin_y = origin[1]


class Controller():
    def __init__(self):
        rospy.init_node("shortest_distacnce_calculator", anonymous=False)
        self.r = rospy.Rate(1/0.005)
        self.start_x = rospy.get_param("/shortest_distacnce_calculator/x_pos")
        self.start_y = rospy.get_param("/shortest_distacnce_calculator/y_pos")
        self.goal_x = rospy.get_param("/shortest_distacnce_calculator/goal_x")
        self.goal_y = rospy.get_param("/shortest_distacnce_calculator/goal_y")
        f = Read_file()
        # self.x_goal_pixel, self.y_goal_pixel = f.get_goal_pixel()
        # self.x_robot_pixel, self.y_robot_pixel = f.get_robot_pixel(self.start_x, self.start_y)
        # rospy.loginfo(f"%%%%%%%%%%% : {self.x_robot_pixel}")
    def run(self):
        # print("(((((((((((((((((((((((((")
        while(1):
            rospy.loginfo(f"%%%%%%%%%%% : {self.start_x}")
            self.r.sleep()
        

if __name__ == "__main__":

    controller = Controller()
    controller.run()