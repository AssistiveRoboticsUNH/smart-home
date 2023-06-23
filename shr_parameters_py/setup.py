import os
from setuptools import setup
from generate_parameter_library_py.setup_helper import generate_parameter_module
from ament_index_python.packages import get_package_share_directory

generate_parameter_module(
    "shr_parameters",
    os.path.join(get_package_share_directory('shr_parameters'), 'params', 'shr_parameters.yaml'),
)

package_name = 'shr_parameters_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='paul',
    maintainer_email='paulgesel@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
