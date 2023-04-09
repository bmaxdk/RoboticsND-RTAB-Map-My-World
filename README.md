# RoboticsND-RTAB-Map-My-World
RTAB-Map (Real-Time Appearance-Based Mapping) 

2D occupancy grid and 3D octomap from a simulated environment using a robot with the [RTAB-Map](http://wiki.ros.org/rtabmap_ros) package. This project will be using the [`rtabmap`](http://wiki.ros.org/rtabmap_ros) package.

Toe setup the [RTAB-Map setup](http://wiki.ros.org/rtabmap_ros/Tutorials/SetupOnYourRobot), it recommended robot configuration requires:

* A 2D Laser, providing sensor_msgs/LaserScan messages
* Odometry sensors, providing nav_msgs/Odometry messages
* 3D Camera, compatible with openni_launch, openni2_launch or freenect_launch ROS packages

## Sensor update
In my [my_robot.xacro]() added optical camera link for RGB-D camera in URDF file. This extra link and an extra joint to the camera link in order to align the camera image in Gazebo properly with the robot. (The Parent link of `camera_optical_joint` should be properly configured to the original camera link)
```xacro
  <joint name="camera_optical_joint" type="fixed">
    <origin xyz="0 0 0" rpy="-1.5707 0 -1.5707"/>
    <parent link="camera_link"/>
    <child link="camera_link_optical"/>
  </joint>

  <link name="camera_link_optical">
  </link>
```
