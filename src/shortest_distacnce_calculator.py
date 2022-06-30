#!/usr/bin/python3
from turtle import distance
import cv2
import numpy as np
import yaml
from math import sqrt
from cv_bridge import CvBridge
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image

class ReadMap():
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

    def dictance_calc(x1, y1, x2, y2):
        d = sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
        return float(d)

    def go_up(self, start_x, start_y):
        j = start_y
        while(self.free_cells[start_x][j] >0 and self.height-1>j>0):
            j +=1
            flag +=1
        end_point = [start_x, j-1]
        d = abs(j - start_y)
        return end_point, d
    
    def go_down(self, start_x, start_y):
        j = start_y
        while (self.free_cells[start_x][j] == -1 and self.height-1>j>0):
            j-=1
        while(self.free_cells[start_x][j] >0 and self.height-1>j>0):
            j -=1
        end_point = [start_x, j+1]
        d = abs(j - start_y)
        return end_point, d
    
    def go_right(self, start_x, start_y):
        i = start_x
        while(self.free_cells[i][start_y] >0 and self.width-1>i>0):
            i +=1
        end_point = [i-1, start_y]
        d = abs(i - start_x)
        return end_point, d
    
    def go_left(self, start_x, start_y):
        i = start_x
        while (self.free_cells[i][start_y] == -1 and self.width-1>i>0):
            i-=1
        while(self.free_cells[i][start_y] >0 and self.width-1>i>0):
            i -=1
        end_point = [i+1, start_y]
        d = abs(i - start_x)
        return end_point, d

    def create_graph(self, start_x_pixel, start_y_pixel):
        graph={}
        for i in range(0, self.width-1):
            j = 0
            while(j < self.height):
                start_point = [i, j]
                end_point_u, d_u = self.go_up(i, j)
                if d_u > 0:
                    key_u = [start_point, end_point_u]
                    graph[str(key_u)] = d_u
                    j += d_u
                else:
                    j +=1
                # rospy.loginfo(f"%%%%%%%%%%% : {d_u}   {i} {j}")


        for j in range(0, self.height-1):
            i = 0
            while(i < self.width):
                start_point = [i, j]
                end_point_r, d_r = self.go_right(i, j)
                if d_r > 0:
                    key_r = [start_point, end_point_r]
                    graph[str(key_r)] = d_r
                    i += d_r
                else:
                    i+=1
                # rospy.loginfo(f"^^^^^^^ : {d_r}   {i} {j}")
        return graph





class Controller():
    def __init__(self):
        rospy.init_node("shortest_distacnce_calculator", anonymous=False)
        self.r = rospy.Rate(1/0.005)
        self.start_x = rospy.get_param("/shortest_distacnce_calculator/x_pos")
        self.start_y = rospy.get_param("/shortest_distacnce_calculator/y_pos")
        self.goal_x = rospy.get_param("/shortest_distacnce_calculator/goal_x")
        self.goal_y = rospy.get_param("/shortest_distacnce_calculator/goal_y")
        f = ReadMap()
        self.cell_value = f.get_pix_value()
        self.x_goal_pixel, self.y_goal_pixel = f.get_goal_pixel(self.goal_x, self.goal_y)
        self.x_robot_pixel, self.y_robot_pixel = f.get_robot_pixel(self.start_x, self.start_y)
        self.graph = f.create_graph(self.x_robot_pixel, self.y_robot_pixel)
        # rospy.loginfo(f"%%%%%%%%%%% : {self.graph}")
    def run(self):
        # print("(((((((((((((((((((((((((")
        while(1):
            # rospy.loginfo(f"%%%%%%%%%%% : {self.start_x}")
            self.r.sleep()
        

if __name__ == "__main__":

    controller = Controller()
    controller.run()