#!/usr/bin/python3
from turtle import distance
import cv2
import numpy as np
import yaml
from math import sqrt
from queue import PriorityQueue
from cv_bridge import CvBridge
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image

class Graph:
    def __init__(self, num_of_vertices):
        self.v = num_of_vertices
        self.edges = [[-1 for i in range(num_of_vertices)] for j in range(num_of_vertices)]
        self.visited = []
    
    def add_edge(self, u, v, weight):
        self.edges[u][v] = weight
        self.edges[v][u] = weight
    
    def dijkstra(graph, start_vertex, res):
      D = {v:float('inf') for v in range(graph.v)}
      # print(D)
      D[start_vertex] = 0

      pq = PriorityQueue()
      pq.put((0, start_vertex))
      
  
      while not pq.empty():
          (dist, current_vertex) = pq.get()
          graph.visited.append(current_vertex)

          for neighbor in range(graph.v):
              if graph.edges[current_vertex][neighbor] != -1:
                  distance = graph.edges[current_vertex][neighbor]
                  if neighbor not in graph.visited:
                      old_cost = D[neighbor]
                      new_cost = D[current_vertex] + distance
                      if new_cost < old_cost:
                          pq.put((new_cost, neighbor))
                          res[neighbor] = current_vertex
                          D[neighbor] = new_cost
      
      return D, res


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
        self.nodes = {}
        self.pixel_nodes_num = {}

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
            # flag +=1
        mian_d = abs(j - start_y)
        y = start_y+mian_d//2
        end_point = [start_x, y]
        d = abs(y - start_y)
        return end_point, d, mian_d
    
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

        mian_d = abs(i - start_x)
        x = start_x+mian_d//2
        end_point = [x, start_y]
        d = abs(x - start_x)
        return end_point, d, mian_d
    
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

        # horizontal_graph=[]
        # vertical_graph = []
        graph = []
        node_num = 0
        for i in range(0, self.width-1):
            j = 0
            while(j < self.height):
                start_point = [i, j]
                end_point_u, d_u, mian_d = self.go_up(i, j)
                if d_u > 0:
                    d = np.zeros(5)
                    # key_u = [start_point, end_point_u]
                    # graph[str(key_u)] = d_u
                    d[0] = i
                    d[1] = j
                    d[2] = end_point_u[0]
                    d[3] = end_point_u[1]
                    d[4] = d_u
                    graph.append(d)
                    rospy.loginfo(f"%%%%%%%%%%% : {d}")
                    j += mian_d
                    # start_str = str(str(i)+" "+str(j))
                    # end_str = str(str(end_point_u[0])+ " "+str(end_point_u[1]))
                    self.nodes[node_num] = start_point
                    self.nodes[node_num + 1] = end_point_u
                    self.pixel_nodes_num[str(start_point)] = node_num
                    self.pixel_nodes_num[str(end_point_u)] = node_num + 1
                    # end_point_r, d_r = self.go_right(d[2], d[3])
                    node_num += 2


                else:
                    j +=1
                


        for j in range(0, self.height-1):
            i = 0
            while(i < self.width):
                start_point = [i, j]
                end_point_r, d_r, mian_d = self.go_right(i, j)
                if d_r > 0:
                    d = np.zeros(5)
                    # key_r = [start_point, end_point_r]
                    # graph[str(key_r)] = d_r
                    d[0] = i
                    d[1] = j
                    d[2] = end_point_r[0]
                    d[3] = end_point_r[1]
                    d[4] = d_r
                    graph.append(d)
                    rospy.loginfo(f"^^^^^^^ : {d}")
                    i += mian_d

                    # start_str = str(str(i)+" "+str(j))
                    # end_str = str(str(end_point_r[0])+ " "+str(end_point_r[1]))
                    if not str(start_point) in self.pixel_nodes_num:
                        self.nodes[node_num] = start_point
                        self.pixel_nodes_num[str(start_point)] = node_num
                        node_num += 1 
                    if not str(end_point_r) in self.pixel_nodes_num:
                        self.nodes[node_num] = end_point_r
                        self.pixel_nodes_num[str(end_point_r)] = node_num

                        node_num += 1 
                else:
                    i+=1
                
        # rospy.loginfo(f"^^^^^^^^^^ : {self.pixel_nodes_num}")
        g = Graph(node_num)
        rospy.loginfo(f"%%%%%%%%%%% : {50000000000}")
        for i in range(len(graph)):
            start_x = int(graph[i][0])
            start_y = int(graph[i][1])
            end_x = int(graph[i][2])
            end_y = int(graph[i][3])
            cost = graph[i][4]
            start = [start_x, start_y]
            end = [end_x, end_y]
            start_node_num = int(self.pixel_nodes_num[str(start)])
            end_node_num = int(self.pixel_nodes_num[str(end)])
            g.add_edge(start_node_num, end_node_num, cost)
        
        D, pq = g.dijkstra(0, {})
        rospy.loginfo(f"&&&&&&& : {pq}")
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
        self.graph = []
        self.graph = f.create_graph(self.x_robot_pixel, self.y_robot_pixel)
        # rospy.loginfo(f"%%%%%%%%%%% : {self.graph}")
        # rospy.loginfo(f"^^^^^^^^^^ : {len(self.graph)}")
    def run(self):
        # print("(((((((((((((((((((((((((")
        while(1):
            # rospy.loginfo(f"%%%%%%%%%%% : {self.start_x}")
            self.r.sleep()
        

if __name__ == "__main__":

    controller = Controller()
    controller.run()