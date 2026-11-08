from setuptools import find_packages, setup

package_name = 'warehouse_planner'

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
    maintainer='thomas',
    maintainer_email='thomasantonykuzhikandam123@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
      'console_scripts': [
        'mission_control = warehouse_planner.mission_control:main',
        'astar_planner = warehouse_planner.astar_planner:main',
        'path_follower = warehouse_planner.path_follower:main',
        'coverage_tracker = warehouse_planner.coverage_tracker:main',
        'odom_to_tf = warehouse_planner.odom_to_tf:main',
        'coverage_nav2 = warehouse_planner.coverage_nav2:main',
        'coverage_recorder = warehouse_planner.coverage_recorder:main',
        'warehouse_traffic = warehouse_planner.warehouse_traffic:main',
        'ground_truth_localizer = warehouse_planner.ground_truth_localizer:main',
        'llm_commander = warehouse_planner.llm_commander:main',
      ],
    },
    
    
)
