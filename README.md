# Husky_Augmentation
Husky packages integrated with ROS2 that can perform Autonomous Navigation, Mapping and Curb Following

## Installation:

### Dependencies:
1. 
```
mkdir -r husky_ws/src && cd husky_ws/src
git clone 
cd .. && colcon build
```
## Connecting to Husky:
Update Udev rule, Check if product ID, vendor ID and Manufacturer name matches the output from lsusb, if not follow the below steps.

```nano husky_ws/src/husky_bringup/udev/41-clearpath.rules```

Paste the Following Line.

```SUBSYSTEMS=="usb", ATTRS{manufacturer}=="Prolific*", SYMLINK+="prolific prolific_$attr{devpath}", MODE="0666"```ky

To ssh into the Husky:
``` ssh 10.118.46.240 ```

Password: testing123


## Launching the Husky

These Commands launch the Husky Robot with basic teleop capabilities.
### Husky_base
```
ros2 launch husky_bringup accessories.launch.py
ros2 launch husky_base base.launch.py
```
### Ouster LiDAR

```
ros2 launch ouster_ros sensor.launch.xml sensor_hostname:=os-122132000423.local
                           or
ros2 launch ouster_ros sensor.launch.xml sensor_hostname:=10.0.0.32
```

### Microstrain GNSS/IMU

```
ros2 launch microstrain_inertial_driver microstrain_launch.py configure:=true activate:=true
```

Ensure correct port is present in microstrain_driver_common/config/params.yml

### Navigation stack

Launches the nav2 stack on the Husky to perform mapless go to goal and Obtsace avoidance
```
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link map
ros2 launch nav2_bringup navigation_launch.py params_file:=husky_nav2_config.yaml
```

We publish a static transform between the base_link frame and map frame as we do not have a static map outdoors. 
```husky_nav2_config.yaml``` file contains the parameters tuned for the husky. global_costmap has ```rolling_window: true``` due to lack of a static map.

### Mapping

Launch the Husky, sensors and the teleop node. 

RTAB-map:

Cartographer:

LIO-SAM:

LeGO-LOAM and LOAM algorithms were tested in ROS1, ingored as they have no ROS2 packages. 

### Curb Detection

A geometric solution to perform curb Detection from LiDAR data using Point CLoud Library. The detected curb points are used to track a path at a fixed offset from the curb.

Steps involved:
1. Downsample
2. Passthrough filters to crop out the region of interest
3. Statistical outlier removal
4. Normal Detection, followed again by outlier removal
5. The Curb Points have normals which have angles in range (-50,50) with the y axis of base_link frame of the robot. FIlter out all points which do not follow this condition. Remove outiers again.
6. Project the detected curb points onto ground plane (z=0) and shift the points by the user given offset.
7. Compute the centroid of the shifted points and use a PID controller to track the y coordinate (distance from the curb) at the specified offset.





