from setuptools import find_packages, setup

package_name = 'scenario_creator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','drone_msgs'],
    zip_safe=True,
    maintainer='jeggenkemper',
    maintainer_email='jeggenkemper@tudelft.nl',
    description='A python package for ROS2 to create pybullet environments for drone testing',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone_setup = scenario_creator.drone_scenario:main',
        ],
    },
)
