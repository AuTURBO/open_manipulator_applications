## Description
This git is Ros application Git.  
*Hardware Structure  
 - Open Manipulator   
 - OpneCR   
 - RealSense D435   

*Base Software information   
 - reference open manipulator git to control open manipulator.   
   https://github.com/ROBOTIS-GIT/open_manipulator.  
 - reference turtlebot3_slam_3d git to detect object and to get 3D coordination.   
   https://github.com/ROBOTIS-JAPAN-GIT/turtlebot3_slam_3d  

## Environment setting     

* Step0, Set D435 environment.   
http://emanual.robotis.com/docs/en/platform/openmanipulator_x/ros_applications/#ros-applications
```bash
  $ sudo apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE
  $ sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u
  $ sudo apt-get install librealsense2-dev librealsense2-utils ros-kinetic-rgbd-launch
  $ cd ~/catkin_ws/src
  $ git clone https://github.com/intel-ros/realsense.git
  $ cd ~/catkin_ws && catkin_make
```

* Step1, Set open manipulator environment.   
http://emanual.robotis.com/docs/en/platform/openmanipulator/    
```bash
S sudo apt-get install ros-kinetic-ros-controllers ros-kinetic-gazebo* ros-kinetic-moveit* ros-kinetic-industrial-core
$ cd ~/catkin_ws/src/
$ git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
$ git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench.git
$ git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench-msgs.git
$ git clone https://github.com/ROBOTIS-GIT/open_manipulator.git
$ git clone https://github.com/ROBOTIS-GIT/open_manipulator_msgs.git
$ git clone https://github.com/ROBOTIS-GIT/open_manipulator_simulations.git
$ git clone https://github.com/ROBOTIS-GIT/robotis_manipulator.git
$ cd ~/catkin_ws && catkin_make
```

* Step2, Set turtlebot3_slam_3d environment.   
https://github.com/ROBOTIS-JAPAN-GIT/turtlebot3_slam_3d    
```bash
$git clone https://github.com/hyunoklee/darknet_ros.git
you unzip darknet.zip file at darknet directory.   
$git clone https://github.com/AuTURBO/turtlebot3_slam_3d.git
$git clone https://github.com/jsk-ros-pkg/jsk_common.git
$catkin_ws && catkin_make
$sudo apt-get install ros-kinetic-jsk-recognition
$sudo apt-get install ros-kinetic-libsiftfast
$sudo apt-get install ros-kinetic-laser-assembler
$sudo apt-get install ros-kinetic-octomap-server
$sudo apt-get install ros-kinetic-nodelet
$sudo apt-get install ros-kinetic-depth-image-proc
$sudo apt-get install ros-kinetic-jsk-topic-tools
$sudo apt-get install ros-kinetic-rtabmap-ros
```

* Step3, Set modified open manipulator application environment.   .    
```bash
  $ cd ~/catkin_ws/src
  $ git clone https://github.com/ROBOTIS-GIT/open_manipulator_applications.git
  $ cd ~/catkin_ws && catkin_make
```

* Step4, Add open manipulator realsense urdf.       
```bash
$ cp -r ~/catkin_ws/src/open_manipulator_applications/extra_urdf/* ~/catkin_ws/src/open_manipulator/open_manipulator_description/urdf/
```

* Step5, Set turtlebot3 environment.       
http://emanual.robotis.com/docs/en/platform/turtlebot3/pc_setup/  
```bash
$ sudo apt-get install ros-kinetic-joy ros-kinetic-teleop-twist-joy ros-kinetic-teleop-twist-keyboard ros-kinetic-laser-proc ros-kinetic-rgbd-launch ros-kinetic-depthimage-to-laserscan ros-kinetic-rosserial-arduino ros-kinetic-rosserial-python ros-kinetic-rosserial-server ros-kinetic-rosserial-client ros-kinetic-rosserial-msgs ros-kinetic-amcl ros-kinetic-map-server ros-kinetic-move-base ros-kinetic-urdf ros-kinetic-xacro ros-kinetic-compressed-image-transport ros-kinetic-rqt-image-view ros-kinetic-gmapping ros-kinetic-navigation ros-kinetic-interactive-markers
$ cd ~/catkin_ws/src/
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
$ cd ~/catkin_ws && catkin_make
```

## Run 

Click image to link to YouTube video.  

* Run picking Apple  
```bash
$ roslaunch turtlebot3_slam_3d turtlebot3_slam_3d.launch use_zed:=false
$ roslaunch open_manipulator_controller open_manipulator_controller.launch use_platform:=true
$ roslaunch open_manipulator_apple open_manipulator_apple2.launch target_object:=apple use_platform:=true
```
[![Video Label](http://img.youtube.com/vi/YdoxhwN8x-E/0.jpg)](https://youtu.be/YdoxhwN8x-E?t=0s)   

* Run Coffee(TalTal)   
```bash
$ roslaunch open_manipulator_controller open_manipulator_controller.launch use_platform:=true
$ roslaunch open_manipulator_apple open_manipulator_taltal.launch target_object:=apple use_platform:=true 
```
[![Video Label](http://img.youtube.com/vi/lo3kEacdwe0/0.jpg)](https://youtu.be/lo3kEacdwe0?t=0s)   
[![Video Label](http://img.youtube.com/vi/DGHDlGohSLM/0.jpg)](https://youtu.be/DGHDlGohSLM?t=0s)   

* Run picking Apple in Gazebo   
There is some bug.   
```bash
$ roslaunch open_manipulator_gazebo open_manipulator_gazebo_jjapit.launch
$ roslaunch open_manipulator_controller open_manipulator_controller.launch use_platform:=false
$ roslaunch open_manipulator_apple open_manipulator_apple.launch target_object:=apple use_platform:=false
```
    

