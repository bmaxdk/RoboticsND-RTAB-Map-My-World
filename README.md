# RoboticsND-RTAB-Map-My-World
RTAB-Map (Real-Time Appearance-Based Mapping) 

2D occupancy grid and 3D octomap from a simulated environment using a robot with the [RTAB-Map](http://wiki.ros.org/rtabmap_ros) package. This project will be using the [`rtabmap`](http://wiki.ros.org/rtabmap_ros) package.

Toe setup the [RTAB-Map setup](http://wiki.ros.org/rtabmap_ros/Tutorials/SetupOnYourRobot), it recommended robot configuration requires:

* A 2D Laser, providing sensor_msgs/LaserScan messages
* Odometry sensors, providing nav_msgs/Odometry messages
* 3D Camera, compatible with openni_launch, openni2_launch or freenect_launch ROS packages
