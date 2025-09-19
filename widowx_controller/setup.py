from setuptools import setup, find_packages

package_name = 'widowx_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(where='.'),
    package_dir={'': '.'},
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/launch.launch.py',
            'launch/widowx_rs.launch.py',
        ]),
    ],
    install_requires=['setuptools', 'rclpy', 'std_msgs'],
    zip_safe=False,
    description='The widowx_controller package',
    license='BSD',
    entry_points={
        'console_scripts': [
            'widowx_controller = widowx_controller.widowx_controller:main',
        ],
    },
)
