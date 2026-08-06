from glob import glob

from setuptools import find_packages, setup

package_name = 'turtle_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/scripts', glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='tmdwodl12@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_node = turtle_project.camera_node:main',
            'opening_test_node = turtle_project.opening_test_node:main',
            'detector_node = turtle_project.detector_node:main',
            'trap_check_node = turtle_project.trap_check_node:main',
            'robot_agent = turtle_project.robot_agent:main',
            'central_node = turtle_project.central_node:main',
            'db_node = turtle_project.db_node:main',
            'rat_herding_node = turtle_project.rat_herding_node:main',
            'webcam_node = turtle_project.webcam_node:main',
        ],
    },
)
