from setuptools import find_packages, setup

package_name = 'yolo_perception'

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
    maintainer='embedded',
    maintainer_email='embedded@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolo_node = yolo_perception.yolo_node:main',
            'yolo_world_node = yolo_perception.yolo_world_node:main',
            'perception_to_goal_node = yolo_perception.perception_to_goal_node:main',
            'command_parser_node = yolo_perception.command_parser_node:main',
            'semantic_map_node = yolo_perception.semantic_map_node:main',
            'schedule_manager_node = yolo_perception.schedule_manager_node:main',
            'supplement_delivery_node = yolo_perception.supplement_delivery_node:main',
        ],
    },
)
