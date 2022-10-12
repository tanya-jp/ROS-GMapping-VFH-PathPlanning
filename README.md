# ROS-GMapping-VFH-PathPlanning
This project is written using `rospy` and uses the turtlebot in gazebo. It has 3 parts:  

1. Creating a map with `Gmapping` using `lidar` data:  
- Moving the robot using `teleop`:
![Screenshot from 2022-10-11 00-09-31](https://user-images.githubusercontent.com/44570354/194948979-75cd26a8-b319-4d06-a6c7-615bc34ea690.png)  
- Moving the robot based on a script for automatic enviroment discovery:
![Screenshot from 2022-10-11 00-10-14](https://user-images.githubusercontent.com/44570354/194949043-22b80340-780e-4e6a-bac7-6b4044f037de.png)

2. Heading towards an end point using `VFH (Vector Field Histogram)` for avoiding obstacles and finding the way:
![Screenshot from 2022-10-11 00-10-42](https://user-images.githubusercontent.com/44570354/194949117-e2cf4d0b-b16f-4a56-a31d-391f63b87f8e.png)

3. Finding the shortest path from starting point to the goal by making a graph using the saved map of the first part and `Dijkstra's algorithm`.
- Found nodes of the graph:
  <p align="left">
    <img src="https://user-images.githubusercontent.com/72709191/195307850-45625744-4da5-4c7b-9cad-a6b2e2650a28.JPG" width=70% height=70%/>
  </p>
- The coordinates of the pixels of the map image corresponding to the found nodes:
  <p align="left">
    <img src="https://user-images.githubusercontent.com/72709191/195309818-11bd3938-bb13-4f56-a08f-26cdf550157a.JPG" width=70% height=70% />
  </p>
