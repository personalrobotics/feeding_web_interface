#!/usr/bin/env python3
from ada_feeding_msgs.action import MoveTo
from feeding_web_app_ros2_test.MoveToDummy import MoveToDummy
import rclpy
from rclpy.executors import MultiThreadedExecutor


def main(args=None):
    rclpy.init(args=args)

    move_to_resting_position = MoveToDummy("MoveFromMouthToRestingPosition", MoveTo)

    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor()

    rclpy.spin(move_to_resting_position, executor=executor)


if __name__ == "__main__":
    main()
