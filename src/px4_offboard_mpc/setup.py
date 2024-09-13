from setuptools import find_packages, setup

package_name = 'px4_offboard_mpc'

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
    maintainer='mist',
    maintainer_email='josh.holder72@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'px4_offboard_MPC = px4_offboard_MPC.offboard_control_MPC:main',
            'px4_offboard_Scvx = px4_offboard_MPC.offboard_control_scvx:main',
            'px4_offboard_safeLayer = px4_offboard_MPC.offboard_control_safeLayer:main'
        ],
    },
)
