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
                'aruco = agent.aruco:main',
        ],
    },
)
