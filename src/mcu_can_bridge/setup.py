from setuptools import find_packages, setup


package_name = 'mcu_can_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/local_ekf.yaml', 'config/global_localization.yaml',
            'config/wheel_odometry.yaml']),
        ('share/' + package_name + '/launch', ['launch/localization.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='SocketCAN bridge for STM32 chassis feedback.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mcu_can_bridge = mcu_can_bridge.mcu_can_bridge:main',
            'cmd_vel_can_bridge = mcu_can_bridge.cmd_vel_can_bridge:main',
        ],
    },
)
