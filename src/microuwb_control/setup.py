from setuptools import find_packages, setup

package_name = 'microuwb_control'

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
    maintainer='avg_shilp_kid',
    maintainer_email='amogh.singh787.9@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'flight_controller = microuwb_control.flight_controller:main',
            'respawn_guard = microuwb_control.respawn_guard:main',
            'test_setpoint_publisher = microuwb_control.test_setpoint_publisher:main',
            'waypoint_nav = microuwb_control.waypoint_nav:main',
        ],
    },
)
