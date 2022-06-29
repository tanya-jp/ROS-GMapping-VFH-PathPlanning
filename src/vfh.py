#!/usr/bin/python3
from copy import copy, deepcopy
import math
import tf
from math import inf
import rospy
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

import numpy as np
from nav_msgs.msg import Odometry
from final_project.srv import GetDistance, GetDistanceRequest
from scipy.signal import argrelextrema




class Controller:
    
    def __init__(self) -> None:
        
        rospy.init_node("controller" , anonymous=False)
        self.cmd_publisher = rospy.Publisher('/cmd_vel' , Twist , queue_size=10)
        goal_x = rospy.get_param("/vfh/goal_x") 
        goal_y = rospy.get_param("/vfh/goal_y") 

        self.VFH_WINDOW_SIZE = rospy.get_param("/vfh/vfh_window_size")
        self.VFH_L = int(rospy.get_param("/vfh/vfh_l"))
        self.VFH_SECTOR_K = int(rospy.get_param("/vfh/vfh_sector_k"))
        self.VFH_S_MAX = int(rospy.get_param("/vfh/vfh_s_max"))
        self.VFH_THRESHOLD = rospy.get_param("/vfh/vfh_threshold")

        
        self.VFH_A = rospy.get_param("/vfh/vfh_a")
        self.VFH_B = rospy.get_param("/vfh/vfh_b")
        
        self.GOAL_POINT = (goal_x, goal_y)

        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.k_p_teta = 0.7
        self.odom_msg = None
        

    def get_laser_ranges(self):        
        rospy.wait_for_service('/get_distance')

        try:
            req = GetDistanceRequest()
            req.dir_name = ''
            req.obstacle_name = ''
            
            get_data = rospy.ServiceProxy('/get_distance', GetDistance)
            resp = get_data(req)
            if len(resp.ranges) == 0:
                return

            # rospy.loginfo(resp)
            return resp.ranges
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
         
    def find_valleys(self, hk_prime):
        # print(hk_prime)
        valleys_idx = []
        for i in range(len(hk_prime)):
            if hk_prime[i] < self.VFH_THRESHOLD:
                valleys_idx.append(i)
            
        valleys = []
        vall = []

        i = 0
        while i < len(valleys_idx):
            while i < len(valleys_idx) and valleys_idx[i] == (valleys_idx[i-1])%len(hk_prime) + 1:
                # print(f"####{i} {len(valleys_idx)}")
                vall.append(valleys_idx[i])
                i += 1
            
            if len(vall) > 0:
                valleys.append(deepcopy(vall))
                vall.clear()
                if i >= len(valleys_idx):
                    break
            vall.append(valleys_idx[i])
            i += 1
        
        if len(valleys) >= 2:
            val_begin = valleys[0]
            val_end = valleys[len(valleys)-1]
            if (val_end[len(val_end)-1] + 1)%len(hk_prime) == val_begin[0]:
                val_begin = val_end + val_begin  
                valleys[0] = val_begin
                valleys.pop(len(valleys)-1)
            
        # print(f":)))))))))))))))))|| {valleys}")
            
        # print(f"@@@@@@@@@@@@@@@@@@@@{hk_prime}")
        # print(f"@@@@@@@@@@@@@@@@@@@@{valleys}")
        # print(f"@@@@@@@@@@@@@@@@@@@@{type(valleys)}")
        # print("---------------------------------------------------------------------------")
        return valleys
       
    
    
    def get_odom_msg(self):
        self.odom_msg = rospy.wait_for_message("/odom", Odometry)
        # print(f'akhe gozoooo: {self.odom_msg.pose.pose.orientation}')


    def calculate_angle_diff(self, point):
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((
            self.odom_msg.pose.pose.orientation.x,
            self.odom_msg.pose.pose.orientation.y,
            self.odom_msg.pose.pose.orientation.z,
            self.odom_msg.pose.pose.orientation.w
        ))
        # print(f"YAW: {yaw}")

        point_x, point_y = point
        # rospy.loginfo(f"$$$$${point}|{point_x}|{point_y}")
        if point_x - self.odom_msg.pose.pose.position.x == 0:
            if point_y > self.odom_msg.pose.pose.position.y:
                angle_to_point = math.pi/2
            else:
                angle_to_point = 3*math.pi/2
        else:
            angle_to_point = math.atan2(point_y - self.odom_msg.pose.pose.position.y, point_x - self.odom_msg.pose.pose.position.x)
        
        angle_to_point = (2*math.pi + angle_to_point) % (2*math.pi)
        yaw = (2*math.pi + yaw) % (2*math.pi)
        # print(f"koskhol: {self.odom_msg.pose.pose.position} {point}")
        
        if angle_to_point > 0.9* 2 * math.pi and yaw < 0.1*math.pi:
            yaw += 2*math.pi
        diff_angle = angle_to_point - yaw
        if diff_angle > math.pi:
            diff_angle = 2*math.pi - diff_angle
        elif diff_angle < -math.pi:
            diff_angle = 2*math.pi + diff_angle
        
        # rospy.loginfo(f"\n====================================================\nangle_to_point: {math.degrees(angle_to_point)}\nyaw:            {math.degrees(yaw)}\ndiff:          {math.degrees(diff_angle)}")
        return diff_angle

 
    def find_steering_direction(self, valleys):
        self.get_odom_msg()
        angle_diff = self.calculate_angle_diff(self.GOAL_POINT)
        angle_diff = math.degrees(angle_diff)
        
        angle_diff/=5        
        best_valley = None
        min_diff = 100000
        kn = None
        for valley in valleys:
            if int(angle_diff) in valley:
                print(f"oskol2222: angle diff/5:{angle_diff * 5} \n{valley} ")
                return math.radians(angle_diff * 5)
             
            for i in range(len(valley)):
                a = abs(angle_diff - valley[i])
                if a < min_diff:
                    min_diff = a
                    best_valley = valley
                    kn = valley[i]
        
        kf = (kn + self.VFH_S_MAX) % (360/5)
        if len(best_valley) > self.VFH_S_MAX:
            teta = (kn+kf)/2 * 5
            teta %= 360
        else:
            teta = (best_valley[0] + best_valley[len(best_valley)-1]) / 2 * 5
            teta %= 360
            
        print(f"oskol: sterring:{teta} angle diff:{angle_diff * 5} | kn: {kn} kf: {kf}\n{best_valley} ")
        return math.radians(teta)
    

    def calculate_vfh(self, laser_ranges):
        if len(laser_ranges) == 0:
            return
        
        hk = []
        for i in range(0, len(laser_ranges), self.VFH_SECTOR_K):
            m = [ self.VFH_A - self.VFH_B*laser_ranges[j] if laser_ranges[j] != inf else 0 for j in range(i, i+self.VFH_SECTOR_K)]
            hk.append(sum(m)/ self.VFH_SECTOR_K)
        
        hk_prime = []
        # print(f"============|||||=| leb hk:{len(hk)}")
        # print(f"============|||||=| hk:{hk}")
        for j in range(len(hk)):
            # # rospy.loginfo(j)
            v = [hk[k % len(hk)] for k in range(j - self.VFH_L, j + self.VFH_L + 1)]
            # rospy.loginfo(v)
            hk_prime.append(sum(v)/len(v))
        
        
        valleys = self.find_valleys(hk_prime)        
        teta = self.find_steering_direction(valleys)         

        # print(f'laser_range: {laser_ranges}')
        print(f'hk_orime: {hk_prime}')        
        print(f'valleys: {valleys}')        
        return teta

        
    def navigate_to_point(self):
            # rospy.loginfo(ranges)
        while True:
            ranges = self.get_laser_ranges()
            if ranges != None:
                teta = self.calculate_vfh(ranges)
                self.move(teta)

    
    def move(self, teta): 
        print(f"*********************************\n{teta}\n*******************************")
        move_cmd = Twist()
        # move_cmd.angular.z = 0
        move_cmd.linear.x = 0.3

        P_TETA = self.k_p_teta * teta
        move_cmd.angular.z = P_TETA 
        self.cmd_publisher.publish(move_cmd)

    
if __name__ == "__main__":
    controller = Controller()
    controller.navigate_to_point()