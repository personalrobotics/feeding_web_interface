# feeding_web_app_ros2_test

This directory contains all the ROS2 nodes that are used to test the web app. This includes: (1) dummy nodes that have the same interface as robot nodes, but actually just sleep; and (2) nodes for the "Test ROS" page of the web app, which demonstrates how to use our ROS helper functions.

## Setup

See the [`ada_feeding` top-level README for setup instructions](https://github.com/personalrobotics/ada_feeding/blob/amaln/rosdeps/README.md).

## Usage

To launch the dummy nodes, rosbridge, and web_video_server, run `source install/setup.bash; ros2 launch feeding_web_app_ros2_test feeding_web_app_dummy_nodes_launch.xml`.

You can also toggle off certain combinations of dummy nodes with arguments:
- **Don't run motion nodes**: `ros2 launch feeding_web_app_ros2_test feeding_web_app_dummy_nodes_launch.xml run_motion:=false`
- **Don't run the face detection node**: `ros2 launch feeding_web_app_ros2_test feeding_web_app_dummy_nodes_launch.xml :run_face_detection=false`
- **Don't run the food detection node**: `ros2 launch feeding_web_app_ros2_test feeding_web_app_dummy_nodes_launch.xml :run_food_detection=false`
- **Don't run the RealSense node**: `ros2 launch feeding_web_app_ros2_test feeding_web_app_dummy_nodes_launch.xml run_real_sense:=false`

You can also combine any of the above arguments.
