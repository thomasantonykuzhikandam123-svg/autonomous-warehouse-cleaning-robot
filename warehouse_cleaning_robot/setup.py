from setuptools import setup
import os
from glob import glob

package_name = 'warehouse_cleaning_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # tells ROS2 to install all launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        # tells ROS2 to install all description files
        (os.path.join('share', package_name, 'description'),
            glob('description/*')),
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*')),
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thomas',
    maintainer_email='thomas@todo.todo',
    description='Warehouse cleaning robot simulation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)