from setuptools import setup
import os
from glob import glob

package_name = 'pick_and_place'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'srv'), glob('srv/*.srv')),  # 추가: 커스텀 .srv 사용
    ],
    install_requires=[
    'setuptools',
    'pymycobot',     
    ],
    zip_safe=True,
    maintainer='jetcobot',
    maintainer_email='jetcobot@todo.todo',
    description='Pick and Place robot system using AprilTag and MyCobot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = pick_and_place.camera_node:main',
            'detection_node = pick_and_place.detection_node:main',
            'transform_node = pick_and_place.transform_node:main',
            'control_node = pick_and_place.control_node:main',
            'robot1_control_node = pick_and_place.robot1_control_node:main',  # 실행 파일 등록
            'robot2_control_node = pick_and_place.robot2_control_node:main',  # 실행 파일 등록
            'camera_test_node = pick_and_place.camera_test_node:main'
        ],
    },
)
