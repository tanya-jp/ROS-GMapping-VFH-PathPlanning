#!/usr/bin/python3

import rospy
from sensor_msgs.msg import LaserScan
from final_project.srv import GetDistance, GetDistanceResponse

class Distance_calculator():
    def __init__(self) -> None:
        rospy.init_node('distance_calculator_lidar_service', anonymous=True)
        rospy.Subscriber("/scan", LaserScan, self.read_distance)
        s = rospy.Service('/get_distance', GetDistance, self.get_data)

        # rospy.loginfo(":_____________________________")
        self.ranges = []
        rospy.spin()


    def read_distance(self, data):
        self.ranges = [d for d in data.ranges]
        # rospy.loginfo(data.intensities)
        # rospy.loginfo(data.ranges)
        # rospy.loginfo("=======================================")


    def get_data(self, req):
        # rospy.loginfo("KIOOONIII")
        res = GetDistanceResponse()
        res.ranges = self.ranges
        return res

    
dc = Distance_calculator()