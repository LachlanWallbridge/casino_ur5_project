from setuptools import find_packages, setup

package_name = 'cv_subscriber'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/weights', ['cv_subscriber/weights/best.pt']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lachlan',
    maintainer_email='lachlan@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'dice_detector = cv_subscriber.dice_detector_node:main'
        ],
    },
)
