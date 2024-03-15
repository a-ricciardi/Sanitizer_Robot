import os
from glob import glob
from setuptools import setup

package_name = 'task_3'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.pmg'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='agatino',
    maintainer_email='ricciardiagatino@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'localization = task_3.localization:main',
        'route_manager = task_3.route_manager:main'
        ],
    },
)