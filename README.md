# Aruco-tags-
 Build librealsense2 package only
      ```bash
      >git clone https://github.com/IntelRealSense/realsense-ros.git 
      > colcon build --packages-select librealsense2 --cmake-args -DBUILD_EXAMPLES=OFF -DBUILD_WITH_STATIC_CRT=OFF -DBUILD_GRAPHICAL_EXAMPLES=OFF
      >ros2 launch realsense2_camera rs_launch.py
      >ros2 launch realsense2_camera rs_launch.py depth_module.depth_profile:=1280x720x30 pointcloud.enable:=true
      ```
