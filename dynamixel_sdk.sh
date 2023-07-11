cd ~/ros2_ws/src
git clone -b $ROS_DISTRO-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git
cd ~/ros2_ws && colcon build --symlink-install --continue-on-error --packages-skip-build-finished
