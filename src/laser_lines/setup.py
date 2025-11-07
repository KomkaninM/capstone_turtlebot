from setuptools import find_packages, setup

package_name = 'laser_lines'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='KomkaninM',
    maintainer_email='komkanin.m@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'scan_only = laser_lines.scan_only:main',
            'best_angle_scan = laser_lines.laser_with_best_angle:main',
        ],
    },
)
