from setuptools import setup
import os
from glob import glob

package_name = 'ball_chaser'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Student',
    maintainer_email='student@istun.edu.tr',
    description='Ball chaser node',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ball_chaser_node = ball_chaser.ball_chaser_node:main'
        ],
    },
)
