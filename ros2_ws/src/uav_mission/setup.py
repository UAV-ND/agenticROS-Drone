from glob import glob

from setuptools import setup

package_name = 'uav_mission'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='UAV-ND',
    description='MVP mission state machine',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': ['mission_node = uav_mission.mission_node:main',
                                    'fake_die = uav_mission.fake_die:main']},
)
