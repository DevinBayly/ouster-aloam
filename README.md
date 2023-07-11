# A-LOAM
## Mofified to adapt for ouster lidar usage

A-LOAM is an Advanced implementation of LOAM (J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time), which uses Eigen and Ceres Solver to simplify code structure. This code is modified from LOAM and [LOAM_NOTED](https://github.com/cuitaixiang/LOAM_NOTED). This code is clean and simple without complicated mathematical derivation and redundant operations. It is a good learning material for SLAM beginners.

<img src="https://github.com/HKUST-Aerial-Robotics/A-LOAM/blob/devel/picture/kitti.png" width = 55% height = 55%/>

**A-LOAM Modifier:** [Tong Qin](http://www.qintonguav.com), [Shaozu Cao](https://github.com/shaozu)
**A-LOAM Modifier for ouster lidar:** [Hao Yuan](hao.yuan@ouster.io)


## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 16.04 or 18.04.
ROS Kinetic or Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)


### 1.2. **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html).

### 1.3. **PCL**
Follow [PCL Installation](http://www.pointclouds.org/downloads/linux.html).

## 2. Build A-LOAM
Modify the CONFIG MODIFICATION SECTION in the common.h file.
The configs are predefined and uncomment the N_SCANS, VERTICAL_FOV, and SCAN_PERIOD based on
the ouster lidar model.

## 3. Build A-LOAM
Clone the repository and catkin_make:

```
    cd ~/catkin_ws/src
    git clone git@bitbucket.org:ouster_io/a-loam.git
    cd ../
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```

## 4. Record an ouster bag by ouster SDK
Record an ouster bag from a live sensor by ouster SDK. [Recording data](https://github.com/ouster-lidar/ouster_example#recording-data)
Record both point cloud and IMU by command: rosbag record /os_cloud_node/points /os_cloud_node/imu


## 5. Run ouster bag
Download [Ouster Sample Bag](https://ouster.com/resources/lidar-sample-data/) to YOUR_DATASET_FOLDER. 

```
    roslaunch aloam_ouster aloam_ouster.launch
    rosbag play YOUR_DATASET_FOLDER/OUSTER.bag
```

## 6. Run with alive sensor
Run A-LOAM real time with Ouster lidar. Follow the Running ROS Nodes with a [Live Sensor section](https://github.com/ouster-lidar/ouster_example#running-ros-nodes-with-a-live-sensor) in the ouster github.

```
    roslaunch aloam_ouster aloam_ouster.launch
    roslaunch ouster_ros ouster.launch sensor_hostname:=<sensor hostname> \
                                   udp_dest:=<udp data destination> \
                                   metadata:=<path to metadata json> \
                                   lidar_mode:=<lidar mode> viz:=<viz>
```

## 7.Acknowledgements
Thanks for LOAM(J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time) and [LOAM_NOTED](https://github.com/cuitaixiang/LOAM_NOTED).
