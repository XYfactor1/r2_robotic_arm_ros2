from setuptools import setup
from glob import glob
import os

package_name = 'r2_kfs'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ray',
    maintainer_email='ray@example.com',
    description='RealSense YOLO 3D Detection with Advanced Coordinate Transform (v2)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main_mal_3D = r2_kfs.main_mal_3D:main',
        ],
    },
)
