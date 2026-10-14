import os
from setuptools import setup, find_packages

package_name = 'lerobot_decision'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/decision_fsm.launch.py']),
        ('share/' + package_name + '/config', ['config/fsm_config.yaml']),
        (os.path.join('lib', package_name), ['scripts/fsm_node.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='FSM-based decision layer for LeRobot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fsm_node.py = lerobot_decision.fsm_node:main',
        ],
    },
)
