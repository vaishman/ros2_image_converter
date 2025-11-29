from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'image_conversion_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # This line installs your launch files into the package share directory
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='leo',
    maintainer_email='ghousul',
    description='Image conversion node with grayscale/color mode switch',
    license='BSD-3-Clause',  
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_conversion = image_conversion_pkg.image_conversion:main' # this calls the main function from the terminal for ros2
        ],
    },
)

