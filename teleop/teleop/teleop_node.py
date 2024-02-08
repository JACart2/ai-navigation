import rclpy
from motor_control_interface.msg import VelAnglePlanned

class Teleop(rclpy.node.Node):
    def __init__(self):
        super().__init__("motor_endpoint")
        
        self.nav_pub = self.create_publisher(VelAnglePlanned, "/nav_cmd", 10)
        self.timer = self.create_timer(10, self.timer_callback)


    def timer_callback(self):
        pass


def main():
    """
    The main method that actually handles spinning up the node."""

    rclpy.init()
    node = Teleop()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
