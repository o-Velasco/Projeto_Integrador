from setuptools import setup

package_name = 'rise_and_land_test'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='francisco',
    maintainer_email='your_email@example.com',
    description='Autonomous takeoff, hover, and land test for Crazyflie',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rise_and_land_test = rise_and_land_test.rise_and_land_test:main',
        ],
    },
)
