from setuptools import setup
import os
from glob import glob

package_name = 'yolostate'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'yolodata'), glob('yolodata/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ns',
    maintainer_email='ns@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_human_detect=yolostate.yolo_human_detect:main',  #no use, just to copy this file
            'detecthuman = yolostate.detecthuman:main',
            'detect_human_depth = yolostate.detect_human_depth:main',
            'downloadyolo = yolostate.downloadyolo:main',
        ],
    },
)
