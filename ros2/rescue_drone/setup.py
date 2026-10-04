from setuptools import find_packages, setup

package_name = 'rescue_drone'

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
    maintainer='hayat',
    maintainer_email='hayahmam3@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'position_reader = rescue_drone.position_reader:main',
            'flight_controller = rescue_drone.flight_controller:main',
           'person_detector = rescue_drone.person_detector:main',
           'person_tracker = rescue_drone.person_tracker:main',
           'rescue_controller = rescue_drone.rescue_controller:main',
           'rescue_mission = rescue_drone.rescue_mission:main',
           'hover_test = rescue_drone.hover_test:main',
           'depth_reader = rescue_drone.depth_reader:main',
           'person_distance = rescue_drone.person_distance:main',
           'approach_test = rescue_drone.approach_test:main',
        ],
    },
)
