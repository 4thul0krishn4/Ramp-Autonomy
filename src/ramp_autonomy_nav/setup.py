from setuptools import find_packages, setup

package_name = 'ramp_autonomy_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/ramp_mppi_params.yaml', 'config/nav2_params.yaml', 'config/ekf.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kunal Yadav',
    maintainer_email='kunal780954@gmail.com',
    description='Ramp traversal state machine and slope-mode MPPI integration.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_machine_node = ramp_autonomy_nav.state_machine_node:main',
            'nav2_param_switcher_node = ramp_autonomy_nav.nav2_param_switcher_node:main',
            'cmd_vel_watchdog_node = ramp_autonomy_nav.cmd_vel_watchdog_node:main',
        ],
    },
)
