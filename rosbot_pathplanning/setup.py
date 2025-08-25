from setuptools import setup
import os
from glob import glob

package_name = 'rosbot_pathplanning'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ROSbot Developer',
    maintainer_email='user@example.com',
    description='Integrated path planning system for ROSbot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pathplanning = rosbot_pathplanning.pathplanning:main',
            'integrated_demo = rosbot_pathplanning.integrated_pathplanning:main',
        ],
    },
)