from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'navigation_maxime'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
                (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maxime.leriche',
    maintainer_email='maxime.leriche@tparlem03.cpe.lan',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                            'send_target_pos = navigation_maxime.target_pos_cli:main',
                            'compute_nav = navigation_maxime.compute_nav:main',
                            'sec_cmd_vel = navigation_maxime.secured_cmd_vel_pub:main',
        ],
    },
)
