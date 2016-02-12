TurtleBot Aruco Marker Driver
=============================

This project was created as a part of the course "Leistungskurs C++" at the TUM in 2016. The main task was to make a TurtleBot ([Kobuki](http://kobuki.yujinrobot.com/)) recognize and move to different Aruco markers.

## Concept

**Overview** <br>
The robot features a Kinect-like sensor with an integrated RGB camera, that is used to detect markers in an small area in front of it. After the first marker/goal is detected, the robot moves towards it, while avoiding obstacles. Consecutively it will find and move to eight different positions.

**Aruco code detection and goal determination** <br>
For the recognition of the markers, a pre-existting ROS [wrapper](https://github.com/pal-robotics/aruco_ros) for Aruco detection was used. Our job focused on extracting the marker positions and determine a goal in front of it. This is necessary because the robot must not drive directly to the position of the marker because it would hit the obstacle it is mounted on. <br>
The package responsible for this function can be found in the folder ```goalfinder```. The main output is the ```/goals``` topic that contains all possible movement targets.

**Driver** <br>
After determining the next movement goal, the driver code will take care of maneuvering the robot towards it. This is done by using both the [move_base](http://wiki.ros.org/move_base) package and the [actionlib](http://wiki.ros.org/actionlib) package provided by the default ROS installation.  
The self localization needed by these algorithms is handled completely by the [turtlebot_navigation](http://wiki.ros.org/turtlebot_navigation) package. It's included AMCL node will try to determine the exact current location of the robot in a prerecorded map of the room. (In the *Usage* chapter of this readme, you can learn how this map file was created.)  
```move_base```  integrates the *costmap* functionality. Based on the map provided by the AMCL stack, it will link a cost parameter to each location in it. It will be higher, the closer the point is to an obstacle. For dynamic obstacles not represented on the static AMCL map, the laser scanner is used. It will constantly update the costmap with the detected obstacles information.  
With the current position and the costmap, the *global planner* of ```move_base``` can calculate the best path to a target location. This 	trajectory will be followed as well as possible by a *local planner* that defines the commands sent to the robots motors. (```mobile_base```)  
Since we did not use any calibration whatsoever on the RGB camera, the position of the marker will change constantly while driving towards it. Therefore our system will reevaluate the robot's target position periodically (2s) and send it to move base.

## Packages

A list of all packages can be viewed [here](doc/packages.md).


## Usage

Before getting started make sure the environment variables are correctly set. An example would be:
```
source /opt/ros/indigo/setup.bash

export ROS_PACKAGE_PATH=/home/username/repo:$ROS_PACKAGE_PATH
export ROS_WORKSPACE=/home/username/repo

export ROS_MASTER_URI=http://herz-dame.clients.eikon.tum.de:11311
export ROS_HOSTNAME=`hostname`.clients.eikon.tum.de

# Only on robot:
export TURTLEBOT_SERIAL_PORT=/dev/ttyUSB0
export RPLIDAR_SERIAL_PORT=/dev/ttyUSB1
```


### 1. Minimal setup

This launch file starts all thing needed to access the Kobuki subsystem.

```
# On robot:
roscore
roslaunch bringup_launchers normal.launch
```


### 2. SLAM (map generation)

You can skip this point of you already created a map.

```
# On robot:
roslaunch goalfinder slam.launch

# On PC:
roslaunch goalfinder rviz_slam.launch
```

After generation the new map, you need to save it using the following command:

```
rosrun map_server map_saver -f mymap
```

Before continuing, stop all running nodes.

Change the path to the map in ```goalfinder/launch/_amcl.launch```. The parameter is called: ```map_file```.

### 3. Start AMCL with goalfinder

All found markers will be saved by [stamped pose](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html), even if they are not longer visible to the camera.

```
# On robot:
roslaunch goalfinder goalfinder.launch

# On PC:
roslaunch goalfinder rviz_goals.launch
```

### 4. Set initial position

The topic ```/initialpose``` helps AMCL to determine the current position of the robot at startup. This can be set manually or with the **2D Pose Estimate** function of RVIZ (preferred).

You can skip this step if you want, but be aware, the robot needs to drive a few meters to determine its exact location.

### 5. Start driving

```
# On robot:
roslaunch driver_old start.launch
```

##### Notes

- The robot will beep when reaching the final position in front of a marker.
- Simple obstacles can be avoided. They have to be detectable by the laser scanner, meaning not too narrow or small.

## What did not work

- Using a velocity smoother: ```move_base``` was unable to stop at the target location in time, it basically drove beyond it at every try. Unfortunately this could not be fixed in time. But startup launch file is available in ```src/bringup_launchers/launch/smooth.launch```.
- State machine approach, due to lack of time. Including random walk functionality.

## References

These ready-to-use packages where used:

- [RPLIDAR driver wrapper](https://github.com/robopeak/rplidar_ros)
- [Aruco library wrapper](https://github.com/pal-robotics/aruco_ros)

## License

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.