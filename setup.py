import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'pid_tuning'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'bash_scripts'), glob('bash_scripts/*.sh')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Arne Snoeyink',
    maintainer_email='arne.snoeyink@outlook.com',
    description='The pid_tuning package',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tuning_node_de = scripts.tuning_node_de:main',
            'tuning_node_hs = scripts.tuning_node_hs:main',
        ],
    },
)
