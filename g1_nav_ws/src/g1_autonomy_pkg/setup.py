import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'g1_autonomy_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ament index registration
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # package manifest
        ('share/' + package_name, ['package.xml']),
        # launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # config files
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='developer',
    maintainer_email='dev@todo.todo',
    description='Autonomous SLAM and Nav2 stack for the Unitree G1 humanoid robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'g1_tf_bridge = g1_autonomy_pkg.g1_tf_bridge:main',
        ],
    },
)
