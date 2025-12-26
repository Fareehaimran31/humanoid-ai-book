from setuptools import setup

package_name = 'robot_capstone_system'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Student Name',
    maintainer_email='student@university.edu',
    description='Capstone project for autonomous humanoid robot system',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'autonomous_humanoid_node = robot_capstone_system.autonomous_humanoid_node:main',
            'perception_node = robot_capstone_system.perception_node:main',
            'ai_brain_node = robot_capstone_system.ai_brain_node:main',
            'vla_system_node = robot_capstone_system.vla_system_node:main',
        ],
    },
)