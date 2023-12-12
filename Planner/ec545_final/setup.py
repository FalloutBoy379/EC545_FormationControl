from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ec545_final'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('msg/*.msg')),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ansh Mehta',
    maintainer_email='ansh@bu.edu',
    description='Example package with custom ROS2 messages',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spawner= ec545_final.spawner:main',
            'odometryPublisher = ec545_final.odometryPublisher:main',
            'planner = ec545_final.pathPlannerCustom:main',
            'gui = ec545_final.gui:main',
            'glove = ec545_final.gloveSimulator:main',
            'tagToOdo = ec545_final.tagToOdo:main',
            'logger = ec545_final.OdometryLogger:main'
        ],
    },
)