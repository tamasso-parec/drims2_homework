from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'drims2_homework'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'trees'), glob('trees/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kalman',
    maintainer_email='samuele.sandrini@polito.it',
    description='Drims2 Homework Package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'demo_node = drims2_homework.demo_node:main',
            'dice_identification_node = drims2_homework.DiceIdentificationNode:main'

        ],
    },
)
