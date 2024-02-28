#!/usr/bin/env python
from motor_control_interface.msg import VelAngle
import rclpy.node
import time


class MotorEndpointTest(rclpy.node.Node):

    def __init__(self):
        super().__init__("motor_endpoint_test")

        self.timer = self.create_timer(1, self.timer_callback)
        self.pub_motion = self.create_publisher(VelAngle, "/nav_cmd", 10)

    def timer_callback(self):

        self.destroy_timer(self.timer)
        try:
            # This turns the wheel all the way to the right and all the way to the left respectivelly
            planned_turning = VelAngle()
            planned_turning.vel = 0.0
            planned_turning.angle = 0.0
            self.pub_motion.publish(planned_turning)
            time.sleep(2)

            self.turn_wheel_left(planned_turning, True)
            time.sleep(8)

            self.turn_wheel_left(planned_turning, False)

            time.sleep(8)

            self.drive_forward(planned_turning)

            time.sleep(5)
            planned_turning.vel = 0.0
            self.pub_motion.publish(planned_turning)
            self.get_logger().info("### Test Complete")
            exit()

        except Exception as e:
            print(e)
            print("!" * 10)
            print("Something went wrong so stopping everything")
            print("!" * 10)
            planned_turning.vel = 0.0
            planned_turning.angle = 0.0
            self.pub_motion.publish(planned_turning)
            exit()

    def turn_wheel_left(self, planned_turning, right):
        planned_turning.vel = 0.0

        if right:
            planned_turning.angle = -25.0
        else:
            planned_turning.angle = 25.0

        self.pub_motion.publish(planned_turning)

    def drive_forward(self, planned_turning):
        planned_turning.angle = 0.0
        planned_turning.vel = 1.5

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
