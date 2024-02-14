#!/usr/bin/env python
from motor_control_interface.msg import VelAnglePlanned
import rclpy.node
import time


class MotorEndpointTest(rclpy.node.Node):

    def __init__(self):
        super().__init__("motor_endpoint_test")

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):

        self.pub_motion = self.create_publisher(VelAnglePlanned, "/nav_cmd", 10)

        try:
            # This turns the wheel all the way to the right and all the way to the left respectivelly
            planned_turning = VelAnglePlanned()

            self.turn_wheel_left(planned_turning, True)
            time.sleep(1.0)

            self.turn_wheel_left(planned_turning, False)

            time.sleep(1.0)

            self.drive_5_seconds(planned_turning)

            time.sleep(5)
            planned_turning.vel_planned = 0.0
            self.pub_motion.publish(planned_turning)
            self.destroy_timer(self.timer)

        except Exception as e:
            print(e)
            print("!" * 10)
            print("Something went wrong so stopping everything")
            print("!" * 10)
            planned_turning.vel_planned = 0.0
            self.pub_motion.publish(planned_turning)
            self.destroy_timer(self.timer)

    def turn_wheel_left(self, planned_turning, right):
        planned_turning.vel_planned = 0.0

        if right:
            planned_turning.angle_planned = -25.0
        else:
            planned_turning.angle_planned = 25.0

        self.pub_motion.publish(planned_turning)

    def drive_5_seconds(self, planned_turning):
        planned_turning.angle_planned = 0.0
        planned_turning.vel_planned = 1.0

        self.pub_motion.publish(planned_turning)


def main():
    """
    The main method that actually handles spinning up the node."""

    rclpy.init()
    node = MotorEndpointTest()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
