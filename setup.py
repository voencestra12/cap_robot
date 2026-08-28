from glob import glob
from setuptools import find_packages, setup

package_name = 'cap_robot'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/prompts', glob('prompts/*.txt')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/urdf', glob('urdf/*.xacro')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='capstone-team',
    maintainer_email='maintainer@example.com',
    description='ROS2 Humble xArm6 LLM multi-agent and synchronized dual-arm package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'aruco_calib = cap_robot.aruco_calib:main',
            'robot_agent = cap_robot.robot_agent:main',
            'workstation_llm = cap_robot.workstation_llm:main',
            'mount_tf = cap_robot.mount_tf:main',
            'yolo_extra_perception = cap_robot.yolo_extra_perception:main',
        ],
    },
)
