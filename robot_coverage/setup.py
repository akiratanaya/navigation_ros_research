from setuptools import find_packages, setup

package_name = 'robot_coverage'

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
    maintainer='akiratanaya',
    maintainer_email='user@todo.todo',
    description='ROS 2 Python wrapper for Fields2Cover',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'coverage_server = robot_coverage.main:main',
        ],
    },
)