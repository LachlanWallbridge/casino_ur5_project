from setuptools import setup
from glob import glob

package_name = 'ui_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/database', glob('ui_bridge/database/*')),
    ],
    install_requires=['setuptools', 'Flask'],
    zip_safe=True,
    maintainer='lachlan',
    maintainer_email='z5359327@unsw.edu.au',
    description='UI Bridge package exposing Flask API and database for frontend.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'backend_node = ui_bridge.backend_node:main',
        ],
    },
)
