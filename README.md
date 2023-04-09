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

Now configuring the RGB-D camera. In [my_robot.gazebo](), replaced the existing camera and its shared object file to:
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


## Launch File for RTAB-Map `mapping.launch`
This mapping launch file acts as the main node that interfaces with all the required parts to be able to perform SLAM with RTAB-Map into the launch folder.
ros topic required by rtabmap:

* scan
* rgb/image
* depth/image
* rgb/camera_info


