from setuptools import find_packages, setup

package_name = 'robot_station_mqtt'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    # install_requires=['setuptools', 'paho-mqtt'],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yimin_hu',
    maintainer_email='yimin.hu@student.kit.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'door_mqtt_to_joint_state = robot_station_mqtt.door_mqtt_to_joint_state:main',
            'window_mqtt_to_joint_state = robot_station_mqtt.window_mqtt_to_joint_state:main',
            'joint_states_merger = robot_station_mqtt.joint_states_merger:main',
        ],
    },
)
