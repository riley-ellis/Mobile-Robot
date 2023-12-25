from setuptools import setup

package_name = 'robot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='riley',
    maintainer_email='riley@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "laptop_keyboard = robot_controller.listen_keyboard:main",
            "ps4_control = robot_controller.control_kevin:main",
            "imu_node = robot_controller.imu_node:main",
            "car_gps_node = robot_controller.gps_car_node:main",
            "data_collection_node = robot_controller.data_collector_node:main",
            "lidar_scan = robot_controller.lidar_scan_node:main",
            "lidar_to_csv = robot_controller.lidar_to_csv_node:main"
        ],
    },
)
