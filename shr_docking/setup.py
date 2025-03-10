from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'shr_docking'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='akash',
    maintainer_email='moniruzzaman.akash@unh.edu',
    description='This package is built for the docking management',
    license='GPL-3.0 license',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            #Docking server with camera only
            'docking_camera = shr_docking.docking_camera_main:main',
            'docking_camera_server = shr_docking.docking_camera_action:main',

            #Docking server with infrared only
            'docking_ir = shr_docking.docking_ir_main:main',
            'docking_ir_server = shr_docking.docking_ir_action:main',

            #Docking server with camera and infrared combined
            'docking_server = shr_docking.docking_action:main',

            #Docking data manager
            'docking_data_manager = shr_docking.docking_data_manager:main',
        ],
    },
)
