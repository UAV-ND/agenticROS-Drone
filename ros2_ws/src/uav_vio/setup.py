from glob import glob

from setuptools import setup

package_name = 'uav_vio'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='UAV-ND',
    description='cuVSLAM -> MAVROS VIO bridge, FC link launch and status monitor',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vio_mavros_bridge = uav_vio.vio_mavros_bridge:main',
            'fc_status = uav_vio.fc_status_node:main',
            'bridge_replay_check = uav_vio.bridge_replay_check:main',
        ],
    },
)
