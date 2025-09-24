from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'dyer_maker_digital_twin'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*launch.[pxy][ymu]*'))),
        # Include URDF files
        (os.path.join('share', package_name, 'urdf'),
         glob('urdf/*.urdf*') + glob('urdf/*.xacro')),
        # Include world files
        (os.path.join('share', package_name, 'worlds'),
         glob('worlds/*.world') + glob('worlds/*.sdf')),
        # Include model files (only if they exist)
        (os.path.join('share', package_name, 'models'),
         [f for f in glob('models/**/*', recursive=True) if os.path.isfile(f)]),
        # Include config files
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml') + glob('config/*.yml')),
        # Include template files (only if they exist)
        (os.path.join('share', package_name, 'templates'),
         [f for f in glob('templates/**/*', recursive=True) if os.path.isfile(f)]),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        # Removing opencv-python and related deps for now due to conflicts
        # 'opencv-python',
        # 'numpy',
        'PyYAML',
        # 'scipy',
        # 'matplotlib',
    ],
    zip_safe=True,
    maintainer='Dyer-Maker Team',
    maintainer_email='team@dyer-maker.com',
    description='A modular ROS2-based digital twin for resilient manufacturing research',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Only include nodes that actually exist
            'block_spawner = dyer_maker_digital_twin.nodes.block_spawner_node:main',
            # 'vision_node = dyer_maker_digital_twin.nodes.vision_node:main',
            # TODO: Add other entry points as we implement them
        ],
    },
)