from setuptools import setup

package_name = 'pioneer_shr_py'

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
    maintainer='pac48',
    maintainer_email='pac48@wildcats.unh.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['recognize_face_action = pioneer_shr_py.recognize_face_action:main',
                            'read_script_action = pioneer_shr_py.read_script_action:main',
                            'make_call_action = pioneer_shr_py.make_call_action:main',
                            ],
    },
)
