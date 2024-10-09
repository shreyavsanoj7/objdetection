from setuptools import find_packages, setup
import os

from glob import glob

package_name = 'ros2_opencv'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    install_requires=[
        'setuptools',
        'opencv-python-headless',
        'cv_bridge'
    ],    
    data_files=[
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*')),
        (os.path.join('lib', package_name), glob('scripts/*')),
    ],
    zip_safe=True,
    maintainer='shreya',
    maintainer_email='shreya@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher_node = ros2_opencv.cameraPublisher:main',
            'subscriber_node = ros2_opencv.subscriberImage:main',
            
        ],
    },
)

