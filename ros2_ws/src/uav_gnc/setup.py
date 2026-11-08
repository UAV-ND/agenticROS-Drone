from setuptools import setup

package_name = 'uav_gnc'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='UAV-ND',
    description='Flight-controller commander and safety limits',
    license='Apache-2.0',
    tests_require=['pytest'],
)
