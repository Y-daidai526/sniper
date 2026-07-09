from setuptools import setup
import os
from glob import glob

package_name = 'receiver'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, f'{package_name}.proto'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         glob('launch/*.launch.py')),
        ('share/' + package_name + '/config',
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='dev@example.com',
    description='MQTT-based H264 video decoder for RoboMaster referee system',
    license='MIT',
    entry_points={
        'console_scripts': [
            'decoder_node = receiver.video_decoder_node:main',
        ],
    },
)
