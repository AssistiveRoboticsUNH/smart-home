from setuptools import setup
from generate_parameter_library_py.setup_helper import generate_parameter_module

package_name = 'shr_world_state'

# set module_name and yaml file
module_name = "shr_world_state_parameters"
yaml_file = "shr_world_state/shr_world_state_parameters.yaml"
generate_parameter_module(module_name, yaml_file, "")


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/world_state.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pac48',
    maintainer_email='pac48@wildcats.unh.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['world_state_node = shr_world_state.world_state_node:main',
                            'detect_eating_node = shr_world_state.detect_eating_node:main',
                            'detect_taking_pill_node = shr_world_state.detect_taking_pill_node:main',
                            'detect_bed_after_returning_node = shr_world_state.detect_bed_after_returning_node:main',
                            'set_goal_service_node = shr_world_state.set_goal_service:main'
                            ],
    },
)
