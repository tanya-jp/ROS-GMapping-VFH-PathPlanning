#!/usr/bin/python3
from copy import copy, deepcopy
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
        
        self.goal_point = (goal_x, goal_y)


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
        print(hk_prime)
        valleys_idx = []
        for i in range(len(hk_prime)):
            if hk_prime[i] < self.VFH_THRESHOLD:
                valleys_idx.append(i)
            
        valleys = []
        vall = []

        i = 0
        while i < len(valleys_idx):
            while i < len(valleys_idx) and valleys_idx[i] == (valleys_idx[i-1])%len(hk_prime) + 1:
                print(f"####{i} {len(valleys_idx)}")
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
            
        print(f":)))))))))))))))))|| {valleys}")
            
        # print(f"@@@@@@@@@@@@@@@@@@@@{hk_prime}")
        # print(f"@@@@@@@@@@@@@@@@@@@@{valleys}")
        # print(f"@@@@@@@@@@@@@@@@@@@@{type(valleys)}")
        print("---------------------------------------------------------------------------")
        return valleys
        

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
         
    
        
    def navigate_to_point(self):
        while True:
            ranges = self.get_laser_ranges()
            # rospy.loginfo(ranges)
            if ranges != None:
                self.calculate_vfh(ranges)

    

    
if __name__ == "__main__":
    controller = Controller()
    controller.navigate_to_point()