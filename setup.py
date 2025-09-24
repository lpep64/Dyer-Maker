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
        'opencv-python',
        'numpy',
        'PyYAML',
        'scipy',
        'matplotlib',
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='A modular ROS2-based digital twin for resilient manufacturing research',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Core nodes
            'vision_node = dyer_maker_digital_twin.nodes.vision_node:main',
            'pick_place_node = dyer_maker_digital_twin.nodes.pick_place_node:main',
            'sensor_coordinator_node = dyer_maker_digital_twin.nodes.sensor_coordinator_node:main',
            'system_orchestrator_node = dyer_maker_digital_twin.nodes.system_orchestrator_node:main',
            'conveyor_node = dyer_maker_digital_twin.nodes.conveyor_node:main',
            'robot_state_node = dyer_maker_digital_twin.nodes.robot_state_node:main',
            
            # Utility and testing tools
            'test_vision = dyer_maker_digital_twin.utils.test_vision:main',
            'test_robot = dyer_maker_digital_twin.utils.test_robot:main',
            'test_system_integration = dyer_maker_digital_twin.utils.test_system_integration:main',
            'health_monitor = dyer_maker_digital_twin.utils.health_monitor:main',
            'performance_benchmark = dyer_maker_digital_twin.utils.performance_benchmark:main',
            'analyze_performance = dyer_maker_digital_twin.utils.analyze_performance:main',
            
            # Configuration and calibration tools
            'configure_vision = dyer_maker_digital_twin.utils.configure_vision:main',
            'calibrate_robot = dyer_maker_digital_twin.utils.calibrate_robot:main',
            'system_diagnostics = dyer_maker_digital_twin.utils.system_diagnostics:main',
        ],
    },
)