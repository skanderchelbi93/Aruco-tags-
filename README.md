# Aruco-tags
1. Clone this github
2. Build project
PS: Path is where the github repo is cloned.
```bash
cd {Path}/Aruco-tags-/
colcon build --symlink-install
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
source install/setup.bash
ros2 launch realsense2_camera rs_launch.py depth_module.depth_profile:=1280x720x30 pointcloud.enable:=true
```
2. then in new Terminal launch the aruco Tag detector launch file
```bash
source install/setup.bash
ros2 launch opencv_tools opencv_tools aruco_marker_pose_estimation_d435
```
if u want to launch with normal camera:
do calibration first using the chesse.png file in the main (after u print the file):
```bash
python3 camera_calibration.py
```
then launch:
```bash
ros2 launch opencv_tools opencv_tools aruco_marker_pose_estimation_tf
```
to generate Aruco tag:
```bash
python3 generate.py 
```
## As idea Source:
this website was used :
https://automaticaddison.com/how-to-publish-tf-between-an-aruco-marker-and-a-camera/
