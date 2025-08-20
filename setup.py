from setuptools import find_packages, setup

package_name = 'dsg_analyzer'

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
    maintainer='multy',
    maintainer_email='multyxu@gmail.com',
    description='Contain nodes that subscribes to dsg and perform analysis on it',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = dsg_analyzer.test_publisher:main',
            'listener = dsg_analyzer.test_subscriber:main',
        ],
    },
)
