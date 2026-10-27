import os
from glob import glob

from setuptools import setup

package_name = 'bumperbot_supervisor_py'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO',
    maintainer_email='TODO@example.com',
    description='Mission supervisor (Python): Including mission_commander and navigation_health_monitor',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts':[
    'mission_commander = bumperbot_supervisor_py.mission_commander:main',
    'navigation_health_monitor = bumperbot_supervisor_py.navigation_health_monitor:main',
    ]
    },
)
