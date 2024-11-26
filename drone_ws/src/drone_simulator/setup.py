from setuptools import find_packages, setup

package_name = 'drone_simulator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Antreas Kourris',
    maintainer_email='a.kourris@students.tudelft.nl',
    description='A simple ROS2 wrapper over the pybullet drone simulator to control the drone with velocity messages',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone_simulator = drone_simulator.drone_simulator:main'
        ],
    },
)
