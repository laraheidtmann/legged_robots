from setuptools import find_packages, setup

package_name = 'teleoperation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/'+package_name+ '/launch/',['launch/t3_talos_rviz.launch.py']),

    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='laraheidtmann',
    maintainer_email='ge78puq@mytum.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            't3=teleoperation.t3_main:main',
            'marker=teleoperation.tele_robot:main',
        ],
    },
)
