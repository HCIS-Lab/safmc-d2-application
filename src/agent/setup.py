import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'agent'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # (os.path.join('share', package_name), glob('launch/*.launch.py')), TODO
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.yaml*'))),
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='efliao@cs.nycu.edu.tw',
    description='ROS 2 package for controlling the drone in the SAFMC D2 competition. Runs on the drone\'s Raspberry Pi, handling flight control and mission execution.',
    license='Internal Use Only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'agent = agent.agent:main',
        ],
    },
)
