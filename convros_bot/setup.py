from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'convros_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='akash',
    maintainer_email='moniruzzaman.akash@unh.edu',
    description='Package to add conversation capability to robot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'speechRecognizer_main = convros_bot.speechRecognizer_main:main',
            'tts_main = convros_bot.tts_main:main',

            'stt_node = convros_bot.stt_node:main',
            'tts_node = convros_bot.tts_node:main',
            'speech_processor = convros_bot.speech_processor:main',

            'conversation_action = convros_bot.conversation_action:main',
            'question_response_action = convros_bot.question_response_action:main',
        ],
    },
)
