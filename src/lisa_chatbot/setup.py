from setuptools import find_packages, setup

package_name = 'lisa_chatbot'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/lisa.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lisa Dev Team',
    maintainer_email='dev@example.com',
    description='Lisa Chatbot ROS2 - Um assistente educacional com STT, LLM e TTS',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'llm_node = lisa_chatbot.llm_node:main',
            'stt_node = lisa_chatbot.stt_node:main',
            'tts_node = lisa_chatbot.tts_node:main',
        ],
    },
)
