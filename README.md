# Aruco-tags
1. Clone this github
2. Build project 
```bash
cd {Path}/Aruco-tags-/
colcon build
```
3. Build librealsense2 package 

```bash
cd Aruco-tags-/src
git clone https://github.com/IntelRealSense/realsense-ros.git
colcon build --packages-select librealsense2 --cmake-args -DBUILD_EXAMPLES=OFF -DBUILD_WITH_STATIC_CRT=OFF -DBUILD_GRAPHICAL_EXAMPLES=OFF
```
for Intel Depth Camera launch:
1. launch camera launch file in Terminal:
```bash
ros2 launch realsense2_camera rs_launch.py
ros2 launch realsense2_camera rs_launch.py depth_module.depth_profile:=1280x720x30 pointcloud.enable:=true
```
2. then in new Terminal launch the aruco Tag detector launch file
```bash
ros2 launch opencv_tools opencv_tools  
```
