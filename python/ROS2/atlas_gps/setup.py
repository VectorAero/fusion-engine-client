from setuptools import setup
from setuptools import find_packages

package_name = 'atlas_gps'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vectoraero',
    maintainer_email='info@vectoraero.com',
    description='ROS2 driver for PointOneNav Atlas GPS',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'atlas = atlas_gps.atlas_publisher:main'
        ],
    },
)
