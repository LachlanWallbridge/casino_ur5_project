import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'brain'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/brain.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lachlan',
    maintainer_email='z5359327@unsw.edu.au',
    description='Brain package controlling game logic and orchestrating nodes.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'brain_node = brain.brain_node:main',
        ],
    },
)