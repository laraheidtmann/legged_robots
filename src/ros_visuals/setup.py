from setuptools import find_packages, setup

package_name = 'ros_visuals'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='devel',
    maintainer_email='devel@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pub = ros_visuals.publisher_node:main',
            'sub = ros_visuals.subscriber_node:main',
            'coordinate_pub = ros_visuals.t11:main',
            't12= ros_visuals.t12:main',
            
        ],
    },
)
