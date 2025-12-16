from setuptools import setup

package_name = 'object_follower'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch folder too
        ('share/' + package_name + '/launch', ['launch/object_follower_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sancross354',
    maintainer_email='your_email@example.com',
    description='Object following ROS2 node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_follower_node = object_follower.object_follower_node:main',
            'object_follower_node_reconstructed = object_follower.object_follower_node_reconstructed:main',
            'object_follower_experiment = object_follower.object_follower_experiment:main',
            'object_tracker_csrt = object_follower.object_tracker_csrt:main',
            'object_selector_gui = object_follower.object_selector_gui:launch_gui',
            'object_selector_node_pyqt5 = object_follower.object_selector_node_pyqt5:launch_gui',
        ],
    },
)
