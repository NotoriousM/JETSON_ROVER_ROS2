from setuptools import setup

package_name = 'robot_control'

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
    maintainer='jetson',
    maintainer_email='jetson@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_controller_node = robot_control.robot_controller_node:main',
            'keyboard_controller_node = robot_control.keyboard_controller_node:main',
            'dualshock4_controller_node = robot_control.dualshock4_controller_node:main',
            'robot_driver_node = robot_control.robot_driver_node:main',
        ],
    },
)
