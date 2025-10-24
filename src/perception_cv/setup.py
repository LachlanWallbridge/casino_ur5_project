from setuptools import setup
import os
from glob import glob

package_name = 'perception_cv'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include YOLO weights
        (os.path.join('share', package_name, 'dice_cv', 'weights'),
         glob('perception_cv/dice_cv/weights/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lachlan',
    maintainer_email='z5359327@unsw.edu.au',
    description='Perception package containing multiple CV nodes',
    license='MIT',
    entry_points={
        'console_scripts': [
            'aruco_cv = perception_cv.aruco_cv.aruco_node:main',
            'dice_cv = perception_cv.dice_cv.dice_node:main',
            'player_cv = perception_cv.player_cv.player_node:main',
            'cup_cv = perception_cv.cup_cv.cup_node:main',
            'fake_camera = perception_cv.scripts.fake_camera:main',
            'aruco_detector_sim = perception_cv.aruco_cv.aruco_detector_sim:main',
        ],
    },
)
