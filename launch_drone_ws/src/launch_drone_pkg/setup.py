from setuptools import setup

package_name = 'launch_drone_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='team301',
    maintainer_email='team301@todo.todo',
    description='ROS2 package to launch a Tello drone using djitellopy',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'launch_drone_node = launch_drone_pkg.launch_drone_node:main',
        ],
    },
)
