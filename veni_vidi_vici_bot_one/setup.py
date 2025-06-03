import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'veni_vidi_vici_bot_one'

# helper to install only files (no dirs)
def files_in(dir_path, pattern):
    return [f for f in glob(os.path.join(dir_path, pattern)) if os.path.isfile(f)]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # resource index
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        # package.xml
        ('share/' + package_name, ['package.xml']),
        # only Python launch scripts
        (os.path.join('share', package_name, 'launch'),
         files_in('launch', '*.launch.py')
         + files_in('launch', '*.launch.xml')),
        # other folders
        (os.path.join('share', package_name, 'description'),
         files_in('description', '*')),
        (os.path.join('share', package_name, 'config'),
         files_in('config', '*')),
        (os.path.join('share', package_name, 'worlds'),
         files_in('worlds', '*')),
        (os.path.join('share', package_name, 'maps'),
         files_in('maps', '*')),
        (os.path.join('share', package_name, 'general'),
         files_in('general', '*')),
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
            'VVV_sm_node = veni_vidi_vici_bot_one.VVV_sm_node_main:main',
            'detect_duplo = veni_vidi_vici_bot_one.detect_duplo:main',
            'cam_process = veni_vidi_vici_bot_one.cam_process:main',
            # 'detect_ball_3d = veni_vidi_vici_bot_one.detect_ball_3d:main',
        ],
    },
)