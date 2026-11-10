from glob import glob

from setuptools import find_packages, setup


package_name = 'mower_nav2'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
	('share/' + package_name + '/launch',glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/behavior_trees', glob('behavior_trees/*.xml')),
        ('share/' + package_name + '/maps', glob('maps/*.*')),
        ('share/' + package_name + '/maps/indoor_map', glob('maps/indoor_map/*.*')),
        ('share/' + package_name + '/maps/indoor_map/pre_ransac_20260814',
         glob('maps/indoor_map/pre_ransac_20260814/*.*')),
        ('share/' + package_name + '/maps/indoor_map/pre_z_filter_20260814',
         glob('maps/indoor_map/pre_z_filter_20260814/*.*')),
        ('share/' + package_name + '/scripts', glob('scripts/*.py')),
        ('share/' + package_name + '/scripts', glob('scripts/*.sh')),
        ('share/' + package_name, ['README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='Fixed-site Nav2 configuration and launch integration for MobileEss.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'field_workflow = mower_nav2.field_workflow:main',
            'livox_obstacle_cloud = mower_nav2.livox_obstacle_cloud:main',
            'collision_gate = mower_nav2.collision_gate:main',
        ],
    },
)
