from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_robot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # --- Eklenen Satırlar ---
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        # ------------------------

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bahadirsandikci', # Kullanıcı adınla değiştirildi
    maintainer_email='bahadir.sandikci@email.com', # TODO: Gerçek e-postanla değiştir
    description='ROS 2 package for BYM412 Homework 2', # Açıklama eklendi
    license='Apache License 2.0', # Lisans eklendi
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ball_chaser = my_robot_bringup.ball_chaser:main',
        ],
    },
)
