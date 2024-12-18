# Assignment2

## 
### 1.Clean the Workspace
Delete build, install, log folders <br/>

### 2.Navigate to the Robot Workspace
```
cd robot_ws/
```
### 3.Build the Packages 
Build the necessary packages using colcon: <br/>
```
colcon build --symlink-install --packages-select msg_srv_action_interface_example topic_service_action_rclpy_example
```
### 4.Run the nodes 
Launch the desired node from the topic_service_action_rclpy_example package: <br/>
```
ros2 run topic_service_action_rclpy_example <node_name>
```

## Add Node
### 1.Navigate to setup.py

~/robot_ws/src/ros2-seminar-examples/topic_service_action_rclpy_example <br/>
setup.py <br/>
```
#!/usr/bin/env python3

import glob
import os

from setuptools import find_packages
from setuptools import setup

package_name = 'topic_service_action_rclpy_example'
share_dir = 'share/' + package_name

setup(
    name=package_name,
    version='0.6.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (share_dir, ['package.xml']),
        (share_dir + '/launch', glob.glob(os.path.join('launch', '*.launch.py'))),
        (share_dir + '/param', glob.glob(os.path.join('param', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Pyo, Darby Lim',
    author_email='passionvirus@gmail.com, routiful@gmail.com',
    maintainer='Pyo',
    maintainer_email='passionvirus@gmail.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='ROS 2 rclpy example package for the topic, service, action',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'argument = topic_service_action_rclpy_example.arithmetic.argument:main',
            'operator = topic_service_action_rclpy_example.arithmetic.operator:main',
            'calculator = topic_service_action_rclpy_example.calculator.main:main',
            'checker = topic_service_action_rclpy_example.checker.main:main',
            'combined = topic_service_action_rclpy_example.combined.combined:main',
            'calpub = topic_service_action_rclpy_example.scripts.calpub:main',  # Add calpub node
            'calsub = topic_service_action_rclpy_example.scripts.calsub:main',  # Add calsub node
        ],
    },
)
```

### 2. Add node to entry_points 
```
entry_points={
        'console_scripts': [
            'argument = topic_service_action_rclpy_example.arithmetic.argument:main',
            'operator = topic_service_action_rclpy_example.arithmetic.operator:main',
            'calculator = topic_service_action_rclpy_example.calculator.main:main',
            'checker = topic_service_action_rclpy_example.checker.main:main',
            'combined = topic_service_action_rclpy_example.combined.combined:main',
            'calpub = topic_service_action_rclpy_example.scripts.calpub:main',  # Add calpub node
            'calsub = topic_service_action_rclpy_example.scripts.calsub:main',  # Add calsub node
        ],
    }
```
### 3. Run Node
(1)Delete build, install, log folders <br/>
(2)Run Node <br/>
```
cd robot_ws/

colcon build --symlink-install --packages-select msg_srv_action_interface_example topic_service_action_rclpy_example

ros2 run topic_service_action_rclpy_example <node_name>
```
