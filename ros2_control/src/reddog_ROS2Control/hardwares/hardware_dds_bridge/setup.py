from setuptools import find_packages, setup

package_name = 'hardware_dds_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bringup.launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='csl',
    maintainer_email='csl@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hardware_manager = hardware_dds_bridge.hardware_manager:main',
        ],
    },
)
