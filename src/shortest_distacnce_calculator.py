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
        self.pgm_file = cv2.imread('/home/tanya/Desktop/catkin_ws/src/final_project/maps/scenario1-2/map.pgm', -1)
        self.resolution = rospy.get_param("/custom_prefix/resolution")
        self.free_thresh = rospy.get_param("/custom_prefix/free_thresh")
        self.occupied_thresh = rospy.get_param("/custom_prefix/occupied_thresh")
        origin = rospy.get_param("/custom_prefix/origin")
        self.origin_x = origin[0]
        self.origin_y = origin[1]

        self.height = self.pgm_file.shape[0]
        self.width = self.pgm_file.shape[1]
        self.free_cells = np.zeros([self.height, self.width])

    def get_pix_value(self):
        for i in range(self.height):
            for j in range(self.width):
                p = (255 - self.pgm_file[i][j]) / 255.0
                if p > self.occupied_thresh:
                    self.free_cells[i][j] = 0
                elif p < self.free_thresh:
                    self.free_cells[i][j] = 1
                else:
                    self.free_cells[i][j] = -1

        return self.free_cells



    def get_robot_pixel(self, x, y):
        x_pixel = int(abs(x-self.origin_x)/self.resolution)
        y_pixel = int(abs(y-self.origin_y)/self.resolution)
        return x_pixel, y_pixel
    
    def get_goal_pixel(self, goal_x, goal_y):
        x_goal_pixel = int(abs(goal_x-self.origin_x)/self.resolution)
        y_goal_pixel = int(abs(goal_y-self.origin_y)/self.resolution)
        return x_goal_pixel, y_goal_pixel

class Controller():
    def __init__(self):
        rospy.init_node("shortest_distacnce_calculator", anonymous=False)
        self.r = rospy.Rate(1/0.005)
        self.start_x = rospy.get_param("/shortest_distacnce_calculator/x_pos")
        self.start_y = rospy.get_param("/shortest_distacnce_calculator/y_pos")
        self.goal_x = rospy.get_param("/shortest_distacnce_calculator/goal_x")
        self.goal_y = rospy.get_param("/shortest_distacnce_calculator/goal_y")
        f = Read_file()
        self.cell_value = f.get_pix_value()
        self.x_goal_pixel, self.y_goal_pixel = f.get_goal_pixel(self.goal_x, self.goal_y)
        self.x_robot_pixel, self.y_robot_pixel = f.get_robot_pixel(self.start_x, self.start_y)
        rospy.loginfo(f"%%%%%%%%%%% : {self.x_robot_pixel}")
        rospy.loginfo(f"%%%%%%%%%%% : {self.y_robot_pixel}")
        rospy.loginfo(f"%%%%%%%%%%% : {self.x_goal_pixel}")
        rospy.loginfo(f"%%%%%%%%%%% : {self.y_goal_pixel}")
        rospy.loginfo(f"%%%%%%%%%%% : {self.cell_value}")
    def run(self):
        # print("(((((((((((((((((((((((((")
        while(1):
            # rospy.loginfo(f"%%%%%%%%%%% : {self.start_x}")
            self.r.sleep()
        

if __name__ == "__main__":

    controller = Controller()
    controller.run()