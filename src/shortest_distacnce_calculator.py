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
        self.graph = []
        self.pq = []

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

    def go_up(self, start_x, start_y, start):
        j = start_y
        if start:
            while(self.free_cells[start_x][j] != 0 and self.height-1>j>0):    
                j +=1
        else:
            while(self.free_cells[start_x][j] >0 and self.height-1>j>0):
                j +=1
        y =  j-1
        if y > start_y:
            end_point = [start_x, y]
            d = abs(y - start_y)
            return end_point, d
        else:
            return 0, 0
    
    def go_down(self, start_x, start_y):
        j = start_y
        while (self.free_cells[start_x][j] == -1 and self.height-1>j>0):
            j-=1
        while(self.free_cells[start_x][j] >0 and self.height-1>j>0):
            j -=1
        end_point = [start_x, j+1]
        d = abs(j - start_y)
        return end_point, d
    
    def go_right(self, start_x, start_y, start=False):
        i = start_x
        if start:
            while(self.free_cells[i][start_y] != 0 and self.width-1>i>0):
                i +=1

        else:
            while(self.free_cells[i][start_y] >0 and self.width-1>i>0):
                i +=1
        x = i-1
        if x > start_x:
            end_point = [x, start_y]
            d = abs(x - start_x)
            return end_point, d
        else:
            return 0, 0
    
    def go_left(self, start_x, start_y):
        i = start_x
        while (self.free_cells[i][start_y] == -1 and self.width-1>i>0):
            i-=1
        while(self.free_cells[i][start_y] >0 and self.width-1>i>0):
            i -=1
        end_point = [i+1, start_y]
        d = abs(i - start_x)
        return end_point, d

    def add_to_graph(self, start_point, end_point, node_num):
        s_p =[]
        s_p.append(int(start_point[0]))
        s_p.append(int(start_point[1]))
        e_p = []
        e_p.append(int(end_point[0]))
        e_p.append(int(end_point[1]))
        if not str(s_p) in self.pixel_nodes_num:
            self.nodes[node_num] = s_p
            self.pixel_nodes_num[str(s_p)] = node_num
            node_num += 1 
        if not str(e_p) in self.pixel_nodes_num:
            self.nodes[node_num] = e_p
            self.pixel_nodes_num[str(e_p)] = node_num
            node_num += 1
        return node_num 

    def create_graph(self, start_x_pixel, start_y_pixel, 
    goal_x_pixel, goal_y_pixel):
        node_num = 0
        start_point = [int(start_x_pixel), int(start_y_pixel)]
        end_point_r, d_r = self.go_right(start_x_pixel, start_y_pixel, 1)
        end_point_u, d_u = self.go_up(start_x_pixel, start_y_pixel, 1)
        if self.width>d_r > 0 and self.height> d_u>0:
                d = np.zeros(5)
                d[0] = int(start_x_pixel)
                d[1] = int(start_y_pixel)
                d[2] = int(end_point_r[0])
                d[3] = int(end_point_u[1])
                end_point = []
                end_point.append(int(d[2]))
                end_point.append(int(d[3]))
                d[4] = d_r + d_u
                self.graph.append(d)
                node_num = self.add_to_graph(start_point, end_point, node_num)
        for i in range(0, self.width-1):
            j =0 
            while(j < self.height):
                start_point = [i, j]
                end_point_u, d_u = self.go_up(i, j, 0)
                if d_u > 0:
                    if d_u > 1:
                        d1 = np.zeros(5)
                        d1[0] = i
                        d1[1] = j
                        d1[2] = int(end_point_u[0])
                        d1[3] = int((end_point_u[1] + j)//2)
                        d1[4] = int(d_u//2)
                        start_point_1 =[]
                        start_point_1.append(d1[0])
                        start_point_1.append(d1[1])
                        end_point_1 = []
                        end_point_1.append(d1[2])
                        end_point_1.append(d1[3])
                        self.graph.append(d1)
                        # rospy.loginfo(f"%%%%%%%%%%%%%%% : {(d1)}")
                        node_num = self.add_to_graph(start_point_1, end_point_1, node_num)
                        
                        d2 = np.zeros(5)
                        d2[0] = d1[2]
                        d2[1] = d1[3]
                        d2[2] = end_point_u[0]
                        d2[3] = end_point_u[1]
                        d2[4] = d_u - d1[4]
                        start_point_2 = (d2[0], d2[1])
                        end_point_2 = (d2[2], d2[3])
                        self.graph.append(d2)
                        node_num = self.add_to_graph(start_point_2, end_point_2, node_num)

                        end_point_r, d_r = self.go_right(int(d1[2]), int(d1[3]), 0)
                        if d_r > 0:
                            d = np.zeros(5)
                            d[0] = int(d1[2])
                            d[1] = int(d1[3])
                            d[2] = int(end_point_r[0])
                            d[3] = int(end_point_r[1])
                            d[4] = int(d_r)
                            start_point_r =[]
                            start_point_r.append(d1[0])
                            start_point_r.append(d1[1])
                            self.graph.append(d)
                            node_num = self.add_to_graph(start_point_r, end_point_r, node_num)
                        j += d_u
                    else:
                        d = np.zeros(5)
                        d[0] = int(i)
                        d[1] = int(j)
                        d[2] = int(end_point_u[0])
                        d[3] = int(end_point_u[1])
                        d[4] = int(d_u)
                        self.graph.append(d)
                        j += d_u
                        node_num = self.add_to_graph(start_point, end_point_u, node_num)
                else:
                    j +=1
                


        for j in range(0, self.height-1):
            i =0
            while(i < self.width):
                start_point = [i, j]
                end_point_r, d_r = self.go_right(i, j, 0)
                if d_r > 0:
                    d = np.zeros(5)
                    d[0] = i
                    d[1] = j
                    d[2] = int(end_point_r[0])
                    d[3] = int(end_point_r[1])
                    d[4] = d_r
                    self.graph.append(d)
                    node_num = self.add_to_graph(start_point, end_point_r, node_num)

                    end_point_u, d_u = self.go_up(int(d[2]), int(d[3]), 0)
                    if d_u > 0:
                            du = np.zeros(5)
                            du[0] = int(d[2])
                            du[1] = int(d[3])
                            du[2] = int(end_point_u[0])
                            du[3] = int(end_point_u[1])
                            du[4] = int(d_u)
                            start_point_u =[]
                            start_point_u.append(du[0])
                            start_point_u.append(du[1])
                            self.graph.append(du)
                            node_num = self.add_to_graph(start_point_u, end_point_u, node_num)
                    i += d_r
                else:
                    i+=1
                
        end_point = [int(goal_x_pixel), int(goal_y_pixel)]
        start_r, d_r = self.go_right(goal_x_pixel, goal_y_pixel, 1)
        start_u, d_u = self.go_up(goal_x_pixel, goal_y_pixel, 1)
        if self.width>d_r > 0 and self.height> d_u>0:
                d = np.zeros(5)
                d[0] = int(start_r[0])
                d[1] = int(start_u[1])
                d[2] = int(goal_x_pixel)
                d[3] = int(goal_y_pixel)
                start_point = []
                start_point.append(int(d[0]))
                start_point.append(int(d[1]))
                d[4] = sqrt((d[0]-d[2])**2 + (d[1]-d[3])**2)
                self.graph.append(d)
                node_num = self.add_to_graph(start_point, end_point, node_num)
        g = Graph(node_num)
        for i in range(len(self.graph)):
            start_x = int(self.graph[i][0])
            start_y = int(self.graph[i][1])
            end_x = int(self.graph[i][2])
            end_y = int(self.graph[i][3])
            cost = self.graph[i][4]
            start = [start_x, start_y]
            end = [end_x, end_y]
            start_node_num = int(self.pixel_nodes_num[str(start)])
            end_node_num = int(self.pixel_nodes_num[str(end)])
    
            g.add_edge(start_node_num, end_node_num, cost)
        
        D, self.pq = g.dijkstra(0, {})
        rospy.loginfo(f"^^^^^^^^^^ : {(self.pq)}")
    
    def find_path(self, goal_x_pixel, goal_y_pixel):
        path = []
        goal = []
        goal.append(int(goal_x_pixel))
        goal.append(int(goal_y_pixel))
        des = int(self.pixel_nodes_num[str(goal)])
        while des != 0:
            des = self.pq[des]
            path.append(des)
            print(path)



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
        f.create_graph(self.x_robot_pixel, self.y_robot_pixel,
                        self.x_goal_pixel, self.y_goal_pixel)
    def run(self):
        while(1):
            self.r.sleep()
        

if __name__ == "__main__":

    controller = Controller()
    controller.run()