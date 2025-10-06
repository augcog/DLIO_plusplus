# DLIO++: A ROS2 variation of Direct LiDAR-Inertial Odometry (DLIO) that fuses gracefully w/o groundtruth GNSS

#### This repo is a fork of the DLIO library: [[Original Repo] (https://github.com/vectr-ucla/direct_lidar_inertial_odometry)]

While the original DLIO implementation has been an excellent light-weight SLAM solution that fuses LiDAR and IMU measurements, it does not directly integrate with existing available GNSS groundtruth and also does not deal with scenarios where GNSS may become denied sporatically in challenging terrain. 

We improved the DLIO implementation with two key features (hence the name DLIO++):

1. We added two dedicated modes: 
   * First mode is a mapping mode where the algorithm still creates light-weight point cloud maps but its world coordinates will be established using an accurate groundtruth GNSS input (preferrably with RTK correction to achieve cm-accuracy). The mapping mode therefore does not need to be performed on-the-go and can be run off-line and away from the vehicle once the needed data bags (e.g., in ROS2 format) are generated and available to use.
   * Second mode is a localization mode where the previously pre-built map is required to perform estimation of the vehicle's location based on the groundtruth GNSS. In localization mode, further alteration of the pre-built map will be disabled so that the vehicle will navigate based on a determinstic map instead of a dyanmically changing map.

## Instructions

### Sensor Setup
In addition to the Sensor Setup from the original DLIO library, DLIO++ was developed using additional compatible sensors:
* LiDAR: Seyond Robin W (https://www.seyond.com/products/robin-w/).
* GNSS: Point One Nav Atlas Dual with Fusion Engine System (https://pointonenav.com/atlas/).

### Dependencies
The following has been verified to be compatible, although other configurations may work too:

- Ubuntu 22.04
- ROS Humble (`rclcpp`, `std_msgs`, `sensor_msgs`, `geometry_msgs`, `nav_msgs`, `pcl_ros`)
- C++ 14
- CMake >= `3.12.4`
- OpenMP >= `4.5`
- Point Cloud Library >= `1.10.0`
- Eigen >= `3.3.7`

```sh
sudo apt install libomp-dev libpcl-dev libeigen3-dev
```

DLIO++ currently is tested to support `ROS 2 Humble`.

### Compiling


### Execution

### Services
To save DLIO's generated map into `.pcd` format, call the following service:

```sh
ros2 service call /save_pcd direct_lidar_inertial_odometry/srv/SavePCD "{'leaf_size': 0.2, 'save_path': '~/map'}"
```

### Test Data


## License
This work is licensed under the terms of the MIT license.