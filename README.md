TurtleBot Aruco Marker Driver
=============================

## Available nodes

A list of all available nodes:


> ### **goalfinder**

>
> Starts camera and aruco marker detection. Provides the position of all found markers and the generated goals.
> 
> ##### Parameter:
> - ```goal_distance_from_marker``` (in meters) <br>  
>   The robot is not moving directly to the position of the marker but before it. ...
> 
> ##### Launch files:
> - ```slam.launch``` <br>  
>   Starts up Kobuki mobile_base, laser scanner and the gmapping tools. It will try to create a precise map of the room with the available distance information. Tip: Move robot around with teleoperation programs.
> 
> - ```rviz_slam.launch``` <br>  
>   Launches a predefined setting for RVIZ suitable for monitoring the SLAM progress.
> 
> - ```goalfinder``` <br>  
>   After the map of the room is available, the goalfinder will start up AMCL, the camera and the Aruco code detection. Poses in front of the markers that directly face it will be published.
> 
> - ```rviz_goals.launch``` <br>  
>   Launches a predefined setting for RVIZ suitable for examination of the goals' position.
>
> -----------------------------
> ### **driver**
> 
> description
> 
> ##### Parameters:
> 
> ...
> 
> ##### Launch files:
> 
> ...
> 



## Usage

Before getting started make sure the environment variables are correctly set. An example would be:
```
source /opt/ros/indigo/setup.bash

export ROS_PACKAGE_PATH=/home/username/repo:$ROS_PACKAGE_PATH 
export ROS_WORKSPACE=/home/username/repo

export ROS_MASTER_URI=http://herz-dame.clients.eikon.tum.de:11311
export ROS_HOSTNAME=`hostname`.clients.eikon.tum.de
```


### 1. SLAM (map generation)

```
On robot:
roscore
roslaunch goalfinder slam.launch

# On PC:
roslaunch goalfinder rviz_slam.launch
```

After generation the new map, you need to save it using the following command:

```
rosrun map_server map_saver -f mymap
```

### 2. Make AMCL use the new generated map

Change the path to the map in ```goalfinder/launch/_amcl.launch```. The parameter is called: ```map_file```.

### 3. Consecutive visits at markers 0-7

```
On robot:
roscore
roslaunch goalfinder goalfinder.launch

# On PC:
roslaunch goalfinder rviz_goals.launch
```

## 4. Set initial position

The topic ```/initialpose``` helps AMCL to determine the current position of the robot at startup. This can be set with the **2D Pose Estimate** function of RVIZ.

## 5. Start driving

```
On robot:
roslaunch driver xyz.launch
```

##### Notes

- The robot should beep when reaching a marker.
- Simple obstacles can be avoided. They have to be detectable by the laser scanner, meaning not too narrow or small.

## Todo

- Marker in rviz mit textur darstellen, damit TF debug entfernt werden kann
- Path.msg entfernen
- Recovery Behaviour
- CLI should use services if possible > response!
- try to disable as much as possible from camera setup (like depth, if we do not use it!)

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
