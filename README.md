# RoboticsND-RTAB-Map-My-World
[image1]: image/a0.png "img1"
[image2]: image/a1.png "img2"
[image3]: image/a2.png "img3"
[image4]: image/a3.png "img4"

[image11]: image/a00.png "img11"
[image22]: image/a11.png "img22"
[image33]: image/a22.png "img33"
[image44]: image/a33.png "img44"

## RTAB-Map (Real-Time Appearance-Based Mapping) 
![alt text][image22]

[image1]: image/RTAB_MAP.gif "Trained Agent"

2D occupancy grid and 3D octomap from a simulated environment using a robot with the [RTAB-Map](http://wiki.ros.org/rtabmap_ros) package. This project will be using the [`rtabmap`](http://wiki.ros.org/rtabmap_ros) package.

Toe setup the [RTAB-Map setup](http://wiki.ros.org/rtabmap_ros/Tutorials/SetupOnYourRobot), it recommended robot configuration requires:

* A 2D Laser, providing sensor_msgs/LaserScan messages
* Odometry sensors, providing nav_msgs/Odometry messages
* 3D Camera, compatible with openni_launch, openni2_launch or freenect_launch ROS packages

![alt text][image11]

#### Build and launch the environment for this setup
With Gazebo and ROS installed, first need to create a catkin workspace.  clone the repository into the src directory:
```bash
$ git clone https://github.com/bmaxdk/RoboticsND-RTAB-Map-My-World.git
$ cd catkin_ws/src
$ catkin_init_workspace
$ cd ..
$ catkin_make
```

Next, you can open Gazebo with the robot in it:
```bash
$ source devel/setup.bash
$ roslaunch my_robot world.launch
```

[Option1] To launch RTAB-Map launch with [localization.launch](https://github.com/bmaxdk/RoboticsND-RTAB-Map-My-World/blob/main/catkin_ws/src/my_robot/launch/localization.launch) file, open a new terminal and execute the following:
```bash
$ source devel/setup.bash
$ roslaunch my_robot localization.launch
```
[Option2] To launch RTAB-Map with [mapping.launch](https://github.com/bmaxdk/RoboticsND-RTAB-Map-My-World/blob/main/catkin_ws/src/my_robot/launch/mapping.launch) and [teleop.launch](https://github.com/bmaxdk/RoboticsND-RTAB-Map-My-World/blob/main/catkin_ws/src/my_robot/launch/teleop.launch)
```bash
$ source devel/setup.bash
$ roslaunch my_robot mapping.launch

# Open New terminal for teleop.launch
$ source devel/setup.bash
$ roslaunch my_robot teleop.launch
```

After save the config of rtabmap, launch Database Viewer
```bash
$ rtabmap-databaseViewer ~/.ros/rtabmap.db
```
* Say yes to using the database parameters
* View -> Constraint View
* View -> Graph View

![alt text][image33]
![alt text][image2]


[rtabmap.db](https://github.com/bmaxdk/RoboticsND-RTAB-Map-My-World/blob/main/rtabmap.db) is located this link.
```bash
 src
├── ball_chaser
│   ├── CMakeLists.txt
│   ├── launch
│   │   └── ball_chaser.launch
│   ├── package.xml
│   ├── src
│   │   ├── drive_bot.cpp
│   │   └── process_image.cpp
│   └── srv
│       └── DriveToTarget.srv
├── CMakeLists.txt -> /opt/ros/noetic/share/catkin/cmake/toplevel.cmake
├── my_robot
│   ├── CMakeLists.txt
│   ├── config
│   │   ├── base_local_planner_params.yaml
│   │   ├── costmap_common_params.yaml
│   │   ├── global_costmap_params.yaml
│   │   ├── local_costmap_params.yaml
│   │   └── __MACOSX
│   ├── launch
│   │   ├── amcl.launch
│   │   ├── localization.launch        #RTAB-Map Localization launchfile
│   │   ├── mapping.launch
│   │   ├── robot_description.launch   #RTAB-Map mapping launch file
│   │   ├── teleop.launch              #teleop.launch file
│   │   └── world.launch
│   ├── maps
│   │   ├── map.pgm
│   │   └── map.yaml
│   ├── meshes
│   │   └── hokuyo.dae
│   ├── package.xml
│   ├── urdf                            # My robot
│   │   ├── my_robot.gazebo
│   │   └── my_robot.xacro       
│   └── worlds
│       ├── cho_robot_world.world       # My current world file
│       └── robotl1_old.world
├── teleop_twist_keyboard
│   ├── CHANGELOG.rst
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── README.md
│   └── teleop_twist_keyboard.py
└── whereami
    ├── CMakeLists.txt
    ├── config
    │   ├── base_local_planner_params.yaml
    │   ├── costmap_common_params.yaml
    │   ├── global_costmap_params.yaml
    │   ├── local_costmap_params.yaml
    │   └── __MACOSX
    ├── launch
    │   └── amcl.launch
    ├── maps
    │   ├── map.pgm
    │   └── map.yaml
    └── package.xml



```
![alt text][image44]
![alt text][image3]


## Sensor update
In my [my_robot.xacro](https://github.com/bmaxdk/RoboticsND-RTAB-Map-My-World/blob/main/catkin_ws/src/my_robot/urdf/my_robot.xacro) added optical camera link for RGB-D camera in URDF file. This extra link and an extra joint to the camera link in order to align the camera image in Gazebo properly with the robot. (The Parent link of `camera_optical_joint` should be properly configured to the original camera link)
```xacro
  <joint name="camera_optical_joint" type="fixed">
    <origin xyz="0 0 0" rpy="-1.5707 0 -1.5707"/>
    <parent link="camera_link"/>
    <child link="camera_link_optical"/>
  </joint>

  <link name="camera_link_optical">
  </link>
```

Now configuring the RGB-D camera. In [my_robot.gazebo](https://github.com/bmaxdk/RoboticsND-RTAB-Map-My-World/blob/main/catkin_ws/src/my_robot/urdf/my_robot.gazebo), replaced the existing camera and its shared object file to:
`libgazebo_ros_camera.so` to that of the Kinect shared object file, `libgazebo_ros_openni_kinect.so`. Updated the `<frameName>` to be the `camera_link_optical` link from abobe in `my_robot.xacro`. On top of this, additional parameters need to be set for the RGB-D camera as well as matching the topics published by the drivers of its real world counterpart.

```xacro
<!-- RGBD Camera -->
  <gazebo reference="camera_link">
    <sensor type="depth" name="camera1">
        <always_on>1</always_on>
        <update_rate>20.0</update_rate>
        <visualize>true</visualize>             
        <camera>
            <horizontal_fov>1.047</horizontal_fov>  
            <image>
                <width>640</width>
                <height>480</height>
                <format>R8G8B8</format>
            </image>
            <depth_camera>

            </depth_camera>
            <clip>
                <near>0.1</near>
                <far>20</far>
            </clip>
        </camera>
         <plugin name="camera_controller" filename="libgazebo_ros_openni_kinect.so">
            <alwaysOn>true</alwaysOn>
            <updateRate>10.0</updateRate>
            <cameraName>camera</cameraName>
            <frameName>camera_link_optical</frameName>                   
            <imageTopicName>rgb/image_raw</imageTopicName>
            <depthImageTopicName>depth/image_raw</depthImageTopicName>
            <pointCloudTopicName>depth/points</pointCloudTopicName>
            <cameraInfoTopicName>rgb/camera_info</cameraInfoTopicName>              
            <depthImageCameraInfoTopicName>depth/camera_info</depthImageCameraInfoTopicName>            
            <pointCloudCutoff>0.4</pointCloudCutoff>                
                <hackBaseline>0.07</hackBaseline>
                <distortionK1>0.0</distortionK1>
                <distortionK2>0.0</distortionK2>
                <distortionK3>0.0</distortionK3>
                <distortionT1>0.0</distortionT1>
                <distortionT2>0.0</distortionT2>
            <CxPrime>0.0</CxPrime>
            <Cx>0.0</Cx>
            <Cy>0.0</Cy>
            <focalLength>0.0</focalLength>
            </plugin>
    </sensor>
  </gazebo>
```


## Launch File for RTAB-Map [mapping.launch](https://github.com/bmaxdk/RoboticsND-RTAB-Map-My-World/blob/main/catkin_ws/src/my_robot/launch/mapping.launch) and [localization.launch](https://github.com/bmaxdk/RoboticsND-RTAB-Map-My-World/blob/main/catkin_ws/src/my_robot/launch/localization.launch)
This mapping launch file acts as the main node that interfaces with all the required parts to be able to perform SLAM with RTAB-Map into the launch folder.
ros topic required by rtabmap:

* scan
* rgb/image
* depth/image
* rgb/camera_info


```xml
<?xml version="1.0" encoding="UTF-8"?>

<launch>
  <!-- Arguments for launch file with defaults provided -->
  <arg name="database_path"     default="rtabmap.db"/>
  <arg name="rgb_topic"   default="/camera/rgb/image_raw"/>
  <arg name="depth_topic" default="/camera/depth/image_raw"/>
  <arg name="camera_info_topic" default="/camera/rgb/camera_info"/>  

  
  <!-- Mapping Node -->
  <group ns="rtabmap">
    <node name="rtabmap" pkg="rtabmap_ros" type="rtabmap" output="screen" args="--delete_db_on_start">

      <!-- Basic RTAB-Map Parameters -->
      <param name="database_path"       type="string" value="$(arg database_path)"/>
      <param name="frame_id"            type="string" value="base_footprint"/>
      <param name="odom_frame_id"       type="string" value="odom"/>
      <param name="subscribe_depth"     type="bool"   value="true"/>
      <param name="subscribe_scan"      type="bool"   value="true"/>

      <!-- RTAB-Map Inputs -->
      <remap from="scan" to="/scan"/>
      <remap from="rgb/image" to="$(arg rgb_topic)"/>
      <remap from="depth/image" to="$(arg depth_topic)"/>
      <remap from="rgb/camera_info" to="$(arg camera_info_topic)"/>

      <!-- RTAB-Map Output -->
      <remap from="grid_map" to="/map"/>

      <!-- Rate (Hz) at which new nodes are added to map -->
      <param name="Rtabmap/DetectionRate" type="string" value="1"/>

      <!-- 2D SLAM -->
      <param name="Reg/Force3DoF" type="string" value="true"/>

      <!-- Loop Closure Detection -->
      <!-- 0=SURF 1=SIFT 2=ORB 3=FAST/FREAK 4=FAST/BRIEF 5=GFTT/FREAK 6=GFTT/BRIEF 7=BRISK 8=GFTT/ORB 9=KAZE -->
      <param name="Kp/DetectorStrategy" type="string" value="0"/>

      <!-- Maximum visual words per image (bag-of-words) -->
      <param name="Kp/MaxFeatures" type="string" value="400"/>

      <!-- Used to extract more or less SURF features -->
      <param name="SURF/HessianThreshold" type="string" value="100"/>

      <!-- Loop Closure Constraint -->
      <!-- 0=Visual, 1=ICP (1 requires scan)-->
      <param name="Reg/Strategy" type="string" value="0"/>

      <!-- Minimum visual inliers to accept loop closure -->
      <param name="Vis/MinInliers" type="string" value="15"/>

      <!-- Set to false to avoid saving data when robot is not moving -->
      <param name="Mem/NotLinkedNodesKept" type="string" value="false"/>
    </node>
  </group>
</launch>
```

### Reference RTAB-Map
[rtabma Advanced Parameter Tuning Tutorial](http://wiki.ros.org/rtabmap_ros/Tutorials/Advanced%20Parameter%20Tuning)

[List of RTAB-Map Parameters](https://github.com/introlab/rtabmap/blob/master/corelib/include/rtabmap/core/Parameters.h)

## RTAB-Map Real Time Visualization
rtabmapviz, which is an additional node for real time visualization of feature mapping, loop closures, and more. It’s not recommended to use this tool while mapping in simulation due to the computing overhead. rtabmapviz is great to deploy on a real robot during live mapping to ensure that you are getting the necessary features to complete loop closures.


To enable it for mapping, add this code snippet to the `mapping.launch` file. This will launch the rtabmapviz GUI and provide you with realtime feature detection, loop closures, and other relevant information to the mapping process.

```xml
<!-- visualization with rtabmapviz -->
    <node pkg="rtabmap_ros" type="rtabmapviz" name="rtabmapviz" args="-d $(find rtabmap_ros)/launch/config/rgbd_gui.ini" output="screen">
        <param name="subscribe_depth"             type="bool" value="true"/>
        <param name="subscribe_scan"              type="bool" value="true"/>
        <param name="frame_id"                    type="string" value="base_footprint"/>

        <remap from="rgb/image"       to="$(arg rgb_topic)"/>
        <remap from="depth/image"     to="$(arg depth_topic)"/>
        <remap from="rgb/camera_info" to="$(arg camera_info_topic)"/>
        <remap from="scan"            to="/scan"/>
    </node>
```
![alt text][image4]

## Mapping: Database Viewer
```bash
$ rtabmap-databaseViewer ~/.ros/rtabmap.db
```
* Say yes to using the database parameters
* View -> Constraint View
* View -> Graph View


## RTAB-Map Localization [localization.launch](https://github.com/bmaxdk/RoboticsND-RTAB-Map-My-World/blob/main/catkin_ws/src/my_robot/launch/localization.launch)
If you desire to perform localization using the map you created, there are only a few changes you need to make. You can start by duplicating your mapping.launch file and renaming the duplicated file tolocalization.launch.

The following changes need to be made to the `localization.launch` file:

Remove the `args="--delete_db_on_start"` from your node launcher since you will need your database to localize too.

Remove the `Mem/NotLinkedNodesKept` parameter

Add the `Mem/IncrementalMemory` parameter of type `string` and set it to `false` to finalize the changes needed to put the robot into localization mode.
