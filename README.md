# order_optimizer

Evaluation ROS package

Yaml files in folders configuration and orders are not provided within repository.

Run node and rviz:

 ```console
 
    ros2 launch order_optimizer order_optimizer_test.launch.py

 ```



Development help:

 ```console

    ros2 topic pub -1 /currentPosition geometry_msgs/msg/PoseStamped "{ pose: {position: {x: 5.0, y: 5.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"

 ```

 ```console

    ros2 topic pub -1 /nextOrder order_optimizer/msg/OrderDemand "{order_id: 1200016, description: 'Special order'}"

 ```


