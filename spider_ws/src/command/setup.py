from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'command'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']), #[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/command_launch.py', 'launch/close_loop_launch.py']),
        # (os.path.join('share', package_name), glob('nn_controller/*.pt')),
        # ('share/' + package_name + '/nn_controller', ['model_1999.pt']),
        (os.path.join('share', package_name, 'models'), glob('models/*.pt')),
    ],
    install_requires=['setuptools', 'pymavlink'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='jetson@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'command_publisher = command.command_node:main',
        'ui_controller = ui_controller.ui_controller:main',
        'imu_reader = imu.imu_node:main',
        'nn_controller = nn_controller.nn_controller_node:main',
        ],
    },
)
