from setuptools import find_packages, setup

package_name = 'ramp_autonomy_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kunal Yadav',
    maintainer_email='kunal780954@gmail.com',
    description='Stability margin, wheel-slip monitoring, and descent braking control.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stability_monitor_node = ramp_autonomy_control.stability_monitor_node:main',
            'descent_brake_node = ramp_autonomy_control.descent_brake_node:main',
            'motion_stall_detector_node = ramp_autonomy_control.motion_stall_detector_node:main',
            'payload_delivery_node = ramp_autonomy_control.payload_delivery_node:main',
        ],
    },
)
