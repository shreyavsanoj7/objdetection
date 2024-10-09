rm build/ install/ log/ -r
colcon build --cmake-args -Wno-dev &&  
source install/setup.bash &&
ros2 launch ros2_opencv ros.launch.py
