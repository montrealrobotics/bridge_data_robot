from setuptools import setup, find_packages

package_name = 'multicam_server'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/cameras.launch.py',
            'launch/streamer.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO',
    maintainer_email='TODO',
    description='Multicam server package for camera streaming',
    license='BSD',
    entry_points={
        'console_scripts': [
            'start_streamers = multicam_server.start_streamers:main',
            'streamer = multicam_server.streamer:main',
        ],
    },
)
