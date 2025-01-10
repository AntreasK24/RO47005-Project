from setuptools import find_packages, setup

package_name = 'trajectory_generator'

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
    maintainer='Mukil Saravanan',
    maintainer_email='mukilsaravanan@tudelft.nl',
    description='A simple trajectory generator that publishes trajectory at specific rate',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_publisher = trajectory_generator.trajectory_publisher:main',
        ],
    },
)
