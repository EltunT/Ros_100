from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ros_100'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')), 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eva',
    maintainer_email='ohrimenkoed@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_publisher = ros_100.cmd_vel_publisher:main',
            'roboscript = ros_100.roboscript:main',
            'visualisation_and_prediction = ros_100.visualisation_and_prediction:main',
        ],
    },
)
