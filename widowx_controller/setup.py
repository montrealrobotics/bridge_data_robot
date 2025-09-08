from setuptools import setup

package_name = 'widowx_controller'

setup(
    name=package_name,
    version='1.0.0',
    packages=['widowx_controller'],
    package_dir={'': 'src'},
    install_requires=['setuptools'],
    zip_safe=True,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    description='The widowx_controller package',
    license='BSD',
)
