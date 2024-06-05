from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os


def generate_launch_description():

    package_name = 'order_optimizer'
    config = os.path.join(
        FindPackageShare(package_name).find(package_name),
        'config',
        'default_config.yaml' #'bins_vision_data.yaml'
        )
    
    database_path = os.path.join(
        FindPackageShare(package_name).find(package_name),
        'test_data'
        )



    order_optimizer_node = Node(
        package ='order_optimizer',
        executable ='order_optimizer_node',
        name ='OrderOptimizer',
        parameters=[config,{"local_system.database_path": database_path}]
    )


    


    return LaunchDescription([
        order_optimizer_node
        ])