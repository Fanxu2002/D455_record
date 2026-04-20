````md id="umdl2v"
# D455_record

## Description

This ROS2 package is designed for recording **RGB**, **Depth**, and **IMU** data from the Intel RealSense D455 camera.

Point cloud recording is not included in this package.

The package subscribes to topics published by `realsense-ros` and saves sensor data for later processing such as RGB-D reconstruction, SLAM, calibration, or dataset generation.

---

## Dependency

Install `realsense-ros` first:

https://github.com/IntelRealSense/realsense-ros

---

## Features

- Record RGB stream as video (`rgb_video.avi`)
- Save raw depth frames as **16-bit PNG**
- Save IMU data as CSV
- Save timestamps for RGB / Depth frames
- Save RGB / Depth camera intrinsic parameters
- Configurable ROS2 topics and output directory

---

## Default Topics

```text
/camera/camera/color/image_raw
/camera/camera/depth/image_rect_raw
/camera/camera/imu
/camera/camera/color/camera_info
/camera/camera/depth/camera_info
````

---

## Output Structure

```text
output_dir/
├── rgb_video.avi
├── timestamps.csv
├── color_camera_info.json
├── depth_camera_info.json
├── imu/
│   └── imu.csv
└── depth_raw/
    └── *.png
```

---

## Build

```bash
cd ~/realsense_ros2
colcon build --packages-select capture_data
source install/setup.bash
```

---

## Run

```bash
ros2 run capture_data realsense_data_exporter
```

Custom output directory:

```bash
ros2 run capture_data realsense_data_exporter --ros-args -p output_dir:=scene01
```

---

## Author

Fanxu

```
```
