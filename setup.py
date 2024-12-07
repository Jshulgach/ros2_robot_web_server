from setuptools import setup

package_name = 'ros2_robot_web_server'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jonathan',
    maintainer_email='jshulgac@andrew.cmu.edu',
    description='ROS2 wrappers for the Robot Web Server',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_server = ros2_robot_web_server.robot_server:main',
        ],
    },
)
