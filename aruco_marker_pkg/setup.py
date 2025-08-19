from setuptools import find_packages, setup

package_name = 'aruco_marker_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ❌ msg 관련 data_files 제거
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='addinnedu',
    maintainer_email='addinnedu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'monitoring_map_gui = aruco_marker_pkg.monitoring_map_gui:main',
            'monitoring_gui_intergrated = aruco_marker_pkg.monitoring_gui_intergrated:main',
            'monitoring_gui_intergrated_matplot = aruco_marker_pkg.monitoring_gui_intergrated_matplot:main',
            'monitoring_gui_intergrated_final = aruco_marker_pkg.monitoring_gui_intergrated_final:main',
            'monitoring_map_gui02 = aruco_marker_pkg.monitoring_map_gui02:main',
            'monitoring_map_gui_LPF = aruco_marker_pkg.monitoring_map_gui_LPF:main',
            'aruco_test = aruco_marker_pkg.aruco_test:main',
            'monitoring_gui_matplot = aruco_marker_pkg.monitoring_gui_matplot:main',
        ],
    },
)
