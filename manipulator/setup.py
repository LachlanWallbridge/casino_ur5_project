from setuptools import setup

package_name = 'manipulator'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lachlan',
    maintainer_email='z5359327@unsw.edu.au',
    description='Manipulator package interfacing with UR5e using ROS 2 actions.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'pick_action_server = manipulator.pick_action_server:main',
        ],
    },
)
