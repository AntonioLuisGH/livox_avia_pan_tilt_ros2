from setuptools import setup
import os
from glob import glob

package_name = 'simple_pan_tilt'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='antonio',
    maintainer_email='antonio@todo.todo',
    description='Simple Pan/Tilt TF Broadcaster',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'broadcaster = simple_pan_tilt.imu_tf_broadcaster:main',
        ],
    },
)