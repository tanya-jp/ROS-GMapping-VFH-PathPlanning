# ROS-GMapping-VFH-PathPlanning
This project is written using `rospy` and uses the turtlebot in gazebo. It has 3 parts:  

1. Creating a map with `Gmapping` using `lidar` data:  
- Moving the robot using `teleop`:
![Screenshot from 2022-10-11 00-09-31](https://user-images.githubusercontent.com/44570354/194948979-75cd26a8-b319-4d06-a6c7-615bc34ea690.png)  
- Moving the robot based on a script for automatic enviroment discovery:
![Screenshot from 2022-10-11 00-10-14](https://user-images.githubusercontent.com/44570354/194949043-22b80340-780e-4e6a-bac7-6b4044f037de.png)

2. Heading towards an end point using `VFH (Vector Field Histogram)` for avoiding obstacles and finding the way:
![Screenshot from 2022-10-11 00-10-42](https://user-images.githubusercontent.com/44570354/194949117-e2cf4d0b-b16f-4a56-a31d-391f63b87f8e.png)

3. Local path planning & Global path planning  
4. 

