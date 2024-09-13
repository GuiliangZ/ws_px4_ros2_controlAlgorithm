from setuptools import find_packages, setup
import os
from glob import glob
from setuptools import setup

package_name = 'px4_offboard_mpc'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name), glob('resource/*rviz'))
        # (os.path.join('share', package_name), ['scripts/TerminatorScript.sh'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mist',
    maintainer_email='josh.holder72@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'px4_offboard_mpc = px4_offboard_mpc.offboard_control_mpc:main',
            'px4_offboard_scvx = px4_offboard_mpc.offboard_control_scvx:main',
            'px4_offboard_safeLayer = px4_offboard_mpc.offboard_control_safeLayer:main',
            'processes = px4_offboard_mpc.processes:main',
            'px4_attitude_plot = px4_offboard_mpc.px4_attitude:main',
            'scvx_class = px4_offboard_mpc.scvx_class:main'
        ],
    },
)
