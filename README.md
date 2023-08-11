# Husky_Augmentation
Husky packages integrated with ROS2 that can perform Autonomous Navigation, Mapping and Curb Following, Object Detection and Semantic segmentation

## Installation:

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

### Docker 

```
docker run -it --network=host --device=/dev/prolific --device=/dev/video0 --device=/dev/video1 --device=/dev/video2 --device=/dev/video3 --device=/dev/video4 --device=/dev/video5 --device=/dev/input/event18  husky_aug 
```

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

## Navigation stack

Launches the nav2 stack on the Husky to perform mapless go to goal and Obtsace avoidance
```
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link map
ros2 launch nav2_bringup navigation_launch.py params_file:=husky_nav2_config.yaml
```

We publish a static transform between the base_link frame and map frame as we do not have a static map outdoors. 
```husky_nav2_config.yaml``` file contains the parameters tuned for the husky. global_costmap has ```rolling_window: true``` due to lack of a static map.

## Mapping

Launch the Husky, sensors and the teleop node. You dcan use either RTABmap or LIO-SAM. LIO-SAM is better.

RTAB-map:

Installation instructions can bge found [here](https://github.com/introlab/rtabmap_ros) .
```
ros2 launch husky_base ouster.launch.py 
```

LIO-SAM:

Installation instructions can be found [here](https://github.com/TixiaoShan/LIO-SAM).

```
ros2 launch lio_sam run.launch.py 
```
## Sensor Calibration

[Open Calib](https://github.com/PJLab-ADG/SensorsCalibration) was used to perform extrinsic calibration.

Camera intrinsics were directly obtained from realsense camera_info topic.
fx=907.704406738281; fy=907.281127929688; cx=634.006408691406; cy=359.488098144531;

### LiDAR-Camera Calibration
Calibrated Parameters:
rx=0.0033801; ry=-0.7854413; rz=1.5752593; tx=-0.121; ty=-0.033; tz=-0.127;

### LiDAR-IMU Calibration
Calibrated Parameters:
rx=-0.0010833; ry=-0.7858898; rz=-0.0026135; tx=-0.119; ty=0.016; tz=-0.103

## Perception
### Curb Following
A geometric solution to perform curb Detection from LiDAR data using Point CLoud Library. The detected curb points are used to track a path at a fixed offset from the curb.

Steps involved:
1. Downsample
2. Passthrough filters to crop out the region of interest
3. Statistical outlier removal
4. Normal Detection, followed again by outlier removal
5. The Curb Points have normals which have angles in range (-50,50) with the y axis of base_link frame of the robot. Filter out all points which do not follow this condition. Remove outiers again.
6. Project the detected curb points onto ground plane (z=0) and shift the points by the user given offset.
7. Compute the centroid of the shifted points and use a PID controller to track the y coordinate (distance from the curb) at the specified offset.

Set the offset that you nee. The robot has to positioned such that the distance from the curb is as close to the offset.
```
ros2 run husky_perception curb_node 
```
### LiDAR-Camera Fusion:
Use the extrinsic parameters from calibration to fuse the LiDAR and camera. Run the following command to project LiDAR points onto the camera. This can be used to get objects that the camera detects. 

```
ros2 run husky_perception fusion_node 
```
While the robot is moving, use below command
```
ros2 run husky_perception fusion_sync_node 
```


### Semantic segmentation

Uses Nvidia Jetson Infernce package to perform semantic segmentation. This runs on the Orin, so the Husky's onboard computer annd the Orin must have the same DOMAIN_ID to share topics.

Installation instructions can be found [here](https://github.com/dusty-nv/ros_deep_learning).

To run the segmenation model on images from intel realsense, run
```
ros2 launch ros_deep_learning seg.ros2.launch
```

### 3D Object Detection

Installation:
1. [Point Pillars](https://github.com/NVIDIA-AI-IOT/CUDA-PointPillars)
2. [ROS bag to KITTI](https://github.com/leofansq/Tools_RosBag2KITTI)
3. [Bounding box visualization tool](https://github.com/NVIDIA-AI-IOT/viz_3Dbbox_ros2_pointpillars)

Uses pointpillars model for 3D LiDAR Object detection. It has not been integrated with ROS yet. Place your poitncloud files in the data folder of CUDA-Pointpillars. 

Ouster PointCloud -> ROS msg -> KITTI format -> TensorRT inference

Inference
```
cd build && ./demo
```
Viusalisation - change paths accordingly
```
python3 viewer.py "/home/user/viz_3Dbbox_ros2_pointpillars/bin/000001.bin" "/home/user/viz_3Dbbox_ros2_pointpillars/txt/000001.txt" "/home/user/viz_3Dbbox_ros2_pointpillars/images/purple.png"
```
All pictures and videos are in the Loahit_Summer2023_Internship folder

## Internship Summary:
* Developed ROS packages (C++ and Python) containerized with Docker for 3D Mapping and Perception.
* Performed sensor fusion on the Husky Robot with wheel odometry, GPS and IMU using an EKF to achieve precise outdoor localization.
* Deployed ROS2 Navigation Stack on the robot for Autonomous Navigation.
* Created robust calibration routines to perform LiDAR-Camera and LiDAR-IMU calibration.
* Performed 3D mapping using SLAM algorithms like LIO-SAM and RTAB-map.
* Building Geometric and Learning based perception algorithms to perform curb detection, 3D object detection(PointPillars) and segmentation (SegNet).




















