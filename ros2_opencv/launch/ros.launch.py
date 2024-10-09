from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory,get_package_prefix


def generate_launch_description():
    pkg_name = 'ros2_opencv'
    share_dir = get_package_share_directory(pkg_name)


    
    pub=Node(
        package=pkg_name,           # Enter the name of your ROS2 package
        executable="cameraPublisher.py",    # Enter the name of your executable
    )
    
    subs=Node(
        package=pkg_name,           # Enter the name of your ROS2 package
        executable="subscriberImage.py",    # Enter the name of your executable
    )
    
    return LaunchDescription([
    	pub,
        subs,
        ])
