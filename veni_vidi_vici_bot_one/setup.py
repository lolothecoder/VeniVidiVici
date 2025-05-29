import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'veni_vidi_vici_bot_one'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'description'), glob('description/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'general'), glob('general/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='marko',
    maintainer_email='marko@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'VVV_sm_node = veni_vidi_vici_bot_one.VVV_sm_node_main:main'
        ],
    },
)
