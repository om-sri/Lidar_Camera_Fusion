# Description
The code implemented in ROS projects a point cloud obtained by a 3D-Lidar sensor on an image from an RGB camera. The example used the ROS package to calibrate a camera and a LiDAR from [lidar_camera_calibration](https://github.com/ankitdhall/lidar_camera_calibration). 

## Interpolated point cloud 
The white dots are the original point cloud of the lidar. The colored dots are the interpolated point cloud.
<p align='center'>
<img width="100%" src="/images/rviz.png"/>
</p>

## Requisites
- ROS [Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) or [Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu)
- [Velodyne](https://github.com/ros-drivers/velodyne) repository
  ```
    cd ~/(your_work_space)/src
    git clone https://github.com/ros-drivers/velodyne.git -b melodic-devel
    cd ..
    catkin_make --only-pkg-with-deps velodyne
  ```
- [PCL](https://pointclouds.org/) (Point Cloud Library) (tested with pcl 1.8)
- [Armadillo](http://arma.sourceforge.net/download.html) (11.0.1 or higher)
  ```
    tar -xvf armadillo-11.1.1.tar.xz
    cd armadillo-11.1.1
    mkdir build
    cd build
    cmake ..
    make
    sudo make install
  ```
## Topics

### Suscribed Topics
*~/pointcloudTopic* Input Point Cloud message. ([sensor_msgs/PointCloud2](http://docs.ros.org/en/lunar/api/sensor_msgs/html/msg/PointCloud2.html))

*~/imageTopic* Input image message. ([sensor_msgs/Image](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html))

### Published Topics
#### Lidar and camera fusion
*~/points2* Output point cloud interpolated. ([sensor_msgs/PointCloud2](http://docs.ros.org/en/lunar/api/sensor_msgs/html/msg/PointCloud2.html))

*~/pcOnImage_image* lidar point cloud projected on input image. ([sensor_msgs/Image](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html))
## Clone repository
```
    cd ~/catkin_ws/src
    git clone https://github.com/om-sri/Lidar_Camera_Fusion
    cd ..
    catkin_make --only-pkg-with-deps lidar_camera_fusion
```

## Ros Launch

You can use this [rosbag](https://drive.google.com/file/d/1kq9jk_hmU2sOuMyih4WXSnvdyIEpkkoa/view?usp=sharing) to test the repository.  Please note that the homogeneous camera-LiDAR matrix has inaccuracies. We are recalibrating the camera and LiDAR to reduce this error.

#### Rosbag play 

```
  rosbag play ~/{your_rosbag path}/cam_lidar00.bag
```
#### Lidar and camera fusion with rosbag
```
  roslaunch lidar_camera_fusion vlp16OnImg_offline.launch 
```
