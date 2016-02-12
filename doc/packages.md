
## Available nodes:


> ### **aruco**
>
> Contains the library used for Aruco marker detection
>
> Source: (https://github.com/pal-robotics/aruco_ros)

<br>

> ### **aruco_msgs**
>
> Contains topic message files.
>
> Source: (https://github.com/pal-robotics/aruco_ros)

<br>

> ### **aruco_ros**
>
> Contains a ROS wrapper for the Aruco marker detection library. Will send messages from *aruco_msgs* package.
>
> Source: (https://github.com/pal-robotics/aruco_ros)

<br>

> ### **bringup_launchers**
>
> A package that contains the launch files to startup the basic functions of the Kobuki robot.
>
> ##### Parameter files:
>  - move_base/
>    - costmap_common_params.yaml
>    - dummy.yaml
>    - dwa_local_planner_params.yaml
>    - global_costmap_params.yaml
>    - global_planner_params.yaml
>    - local_costmap_params.yaml
>    - move_base_params.yaml
>    - navfn_global_planner_params.yaml
>  - velocity_muxer/mux.yaml
>  - velocity_smoother/config.yaml
>
> ##### Launch files:
>
> - ```normal.launch``` <br>  
>   Launches the minimal turtlebot nodes (mobile_base, robot_state_publisher, velocity muxer ...), the kobuki safety controller and move_base.
>
> - ```smooth.launch``` (**Not working properly!**) <br>  
>   Launches *normal.launch* and maps the move_base output to the velocity smoother.

<br>

> ### **cli**
>
> Provides a simple command line tool to perform get and set functions onto the other nodes. Originally created as a debug tool, it works pretty well and is very useful. Additional commands can easily be added.
>


<br>

> ### **driver**
>
> ...
>
> ##### Parameters:
>  ...
>
> ##### Launch files:
>  ...


<br>

> ### **driver_old**
>
> This package was meant to be superseded by the *driver* package, but it was not finished on time. In here the basic steps are executed in a straightforward manner.
>
> ##### Launch files:
>
> - ```start.launch``` <br>  
>   Starts driving the robot from aruco marker 0 to 7. Will perform constant rotation when position of next marker is unknown.


<br>

> ### **friend** (**not available**)
>
> This package was meant to be use the RGB camera to detect a specific object in the environment.
>

<br>

> ### **goalfinder**
>
> Starts camera and aruco marker detection. Provides the position of all found markers and the generated movement targets.
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


<br>

> ### **pathfinder**
>
> Uses goals from *goalfinder* and ```move_base``` to calculate the paths to the goals. These are published in the ```/paths``` topic and used by the ```driver``` node.
>

<br>

> ### **rplidar_ros**
>
> Wrapper for the RPLIDAR laser scanner driver. Provides scan data in ```/scan``` topic.
>
> Source: (https://github.com/robopeak/rplidar_ros)


## Debug nodes:

> ### **amcl_pose_tf_broadcaster**
>
> Broadcasts the amcl pose to the tf system. This should correspond to the position received by the odometry.

<br>

> ### **aruco_tf_broadcaster**
>
> Broadcasts the Aruco marker posittions to the tf system. This is used for visualization in RVIZ.
