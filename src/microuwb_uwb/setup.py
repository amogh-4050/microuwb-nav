from setuptools import find_packages, setup

package_name = 'microuwb_uwb'

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
            'anchor_publisher = microuwb_uwb.anchor_publisher:main',
            'uwb_range_simulator = microuwb_uwb.uwb_range_simulator:main',
            'test_pose_publisher = microuwb_uwb.test_pose_publisher:main',
        ],
    },
)
