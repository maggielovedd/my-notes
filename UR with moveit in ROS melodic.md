# Setup UR with UR driver and use it in moveit

This doc may only apply to the specification below:
* Ubuntu 18.04 
* ROS melodic
* UR CB3 (with software version >= 3.7) or e-Series (software >= 5.1)  (The one I'm using is CB3 3.11)

## 1. install UR driver
Download [UR_Robot_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver).

The simplified instruction is copied here, please use the link for detail.

### Step 1: clone and build the driver  
```
# source global ros
$ source /opt/ros/<your_ros_version>/setup.bash

# create a catkin workspace
$ mkdir -p catkin_ws/src && cd catkin_ws

# clone the driver
$ git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver

# clone fork of the description. This is currently necessary, until the changes are merged upstream.
$ git clone -b calibration_devel https://github.com/fmauch/universal_robot.git src/fmauch_universal_robot

# install dependencies
$ sudo apt update -qq
$ rosdep update
$ rosdep install --from-paths src --ignore-src -y

# build the workspace. We need an isolated build because of the non-catkin library package.
$ catkin_make

# activate the workspace (ie: source it)
$ source devel/setup.bash
```

### Step 2: Setting up a UR robot for ur_robot_driver
For using the *ur_robot_driver* with a real robot you need to install the
**externalcontrol-1.0.4.urcap** which can be found inside the **resources** folder of [UR_Robot_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver).

**Note**: For installing this URCap a minimal PolyScope version of 3.7 or 5.1 (in case of e-Series) is
necessary. For installing the necessary URCap and creating a program, please see the individual tutorials on
how to [setup a CB3 robot](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/ur_robot_driver/doc/install_urcap_cb3.md).

For my UR Network setup:
```
UR Network(Static address):
IP: 192.168.0.2
Mask: 255.255.255.0
Gate: 192.168.0.10

Host(PC's IP for external control):
IP:192.168.0.3
```

### Step 3: Prepare the ROS PC
For using the driver make sure it is installed (either by the debian package or built from source
inside a catkin workspace).

### Step 4: Extract calibration information
Each UR robot is calibrated inside the factory giving exact forward and inverse kinematics. To also
make use of this in ROS, you first have to extract the calibration information from the robot.

For this, there exists a helper script:

    $ roslaunch ur_calibration calibration_correction.launch \
      robot_ip:=<robot_ip> target_filename:="${HOME}/my_robot_calibration.yaml"

For the parameter `robot_ip` insert the IP address on which the ROS pc can reach the robot. As
`target_filename` provide an absolute path where the result will be saved to.
We recommend keeping calibrations for all robots in your organization in a common package. See the
[package's documentation](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/ur_calibration/README.md) for details.

### Step 5: Launch the driver
Once the driver is built and the **externalcontrol** URCap is installed on the
robot, you are good to go ahead starting the driver. (**Note**: We do recommend, though, to extract your robot's
calibration first.)

To actually start the robot driver use one of the existing launch files

    $ roslaunch ur_robot_driver <robot_type>_bringup.launch robot_ip:=192.168.56.101

where **<robot_type>** is one of *ur3, ur5, ur10, ur3e, ur5e, ur10e, ur16e*. Note that in this example we
load the calibration parameters for the robot "ur10_example".

If you calibrated your robot before, pass that calibration to the launch file:

    $ roslaunch ur_robot_driver <robot_type>_bringup.launch robot_ip:=192.168.56.101 \
      kinematics_config:=$(rospack find ur_calibration)/etc/ur10_example_calibration.yaml

If the parameters in that file don't match the ones reported from the robot, the driver will output
an error during startup, but will remain usable.
For more information on the launch file's parameters see its own documentation.

### Step 6: Launch the driver
Once the robot driver is started, load the previous program on the teach pendante as illustrated here this page: [setup a CB3 robot](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/ur_robot_driver/doc/install_urcap_cb3.md)
on the robot panel that will start the *External Control* program node and execute it.
From that moment on the robot is fully functional. 

### Step 7: Press to test the connection

After launch the driver. Press *Stop* (:stop_button:) and the *Play* button (:arrow_forward:) and the ROS driver will reconnect.
Inside the ROS terminal running the driver you should see the output ```Robot ready to receive control commands.```
Otherwise, you are not connected the controller.

### Step 9: Install other packages for control
```
sudo apt-get install ros-melodic-ros-control ros-melodic-ros-controllers
```

### Step 8: You can tested the connection simply by using ```rqt_joint_trajectory_controller``` first
```
sudo apt-get install ros-melodic-rqt-joint-trajectory-controller
rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller
```

In the controller manager, remember to use the correct action server
```bash
/scaled_pos_joint_traj_controller/follow_joint_trajectory
```

**Troubleshooting**
Error | Solution
------------ | -------------
rqt_joint_trajectory_controller() found no plugin matching ‘xxx' | rm ~/.config/ros.org.rqt_gui.ini



### 2. Use it in Moveit
### Step 1: Install Moveit
```sudo apt-get install ros-melodic-moveit```

However! The default moveit config will not work, you should modify it.
Follow this [link](https://www.it610.com/article/1296087580391055360.htm) to modify the files.
And most importtantly! 
```
    <arg name="allow_trajectory_execution" value="true"/>
    <arg name="fake_execution" value="false"/>
 ```
 
 **Troubleshooting**
 1. [ERROR]: Unable to identify any set of controllers that can actuate the specified joints: [ elbow_joint shoulder_lift_joint shoulder_pan_joint wrist_1_joint wrist_2_joint wrist_3_joint ] 
```
Some possible reasons are: 
1. ROS driver is not really connected (check **Step 7: Press to test the connection** in 1. install UR driver) / 
2. Action server is not correct (Follow the setup in 2. Use it in Moveit )

```

### 3. Calibration

#### Camera calibration
#### Hand-eye calibration




