from setuptools import setup

package_name = 'zomby_startup'

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
    maintainer='usurobotics',
    maintainer_email='usurobotics@gmail.com',
    description='This node will be run on the zomby robot to convert cmd_vel to communication on the usb cord from the computer to the arduino',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'zomby_startup = zomby_startup.zomby_startup:main'
        ],
    },
)
