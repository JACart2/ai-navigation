import rclpy
import tf2_geometry_msgs
import sys, select, os
import tty, termios
from motor_control_interface.msg import VelAngle

MAX_LIN_VEL = 30.0  # Max velocity
MAX_ANG_VEL = 30.0  # Max turn angle

LIN_VEL_STEP_SIZE = 0.5  # Velocity incremement amount
ANG_VEL_STEP_SIZE = 1  # Turn incrememnt amount

msg = """
Control Your Golf Cart!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease velocity 
a/d : turn the wheel left/right

space key, s : force stop

CTRL-C to quit

"""

e = """
Communications Failed
"""


class Teleop(rclpy.node.Node):
    """
    Node to control the golf cart manually using keyboard inputs.
    """

    def __init__(self):
        super().__init__("teleop")

        self.settings = termios.tcgetattr(sys.stdin)

        self.nav_pub = self.create_publisher(VelAngle, "/nav_cmd", 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.status = 0
        self.target_linear_vel = 0.0
        self.target_angular_vel = 0.0

    def display(self):
        """Prints the current state and control guide. Acts as a screen refresh."""
        print("\n" * 8)
        print(f"{self.vels(self.target_linear_vel, self.target_angular_vel)}")
        print(f"{msg}", end="")

    def getKey(self):
        """Returns the key entered into stdin as a string."""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ""

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def vels(self, target_linear_vel, target_angular_vel):
        """String formatting method for current state."""
        return "currently:\tlinear vel %s\t angular vel %s " % (
            target_linear_vel,
            target_angular_vel,
        )

    def constrain(self, input, low, high):
        """Helper method to limit a value within a range."""
        if input < low:
            input = low
        elif input > high:
            input = high
        else:
            input = input

        return input

    def checkLinearLimitVelocity(self, vel):
        """Helper method to constrain velocity."""
        vel = self.constrain(vel, -MAX_LIN_VEL, MAX_LIN_VEL)
        return vel

    def checkAngularLimitVelocity(self, vel):
        """Helper method to constrain angle."""
        vel = self.constrain(vel, -MAX_ANG_VEL, MAX_ANG_VEL)
        return vel

    def timer_callback(self):
        """Main loop that gets input, updates state, and publishes the new target."""
        try:
            self.display()
            key = self.getKey()
            if key == "w":
                self.target_linear_vel = self.checkLinearLimitVelocity(
                    self.target_linear_vel + LIN_VEL_STEP_SIZE
                )
            elif key == "x":
                self.target_linear_vel = self.checkLinearLimitVelocity(
                    self.target_linear_vel - LIN_VEL_STEP_SIZE
                )
            elif key == "a":
                self.target_angular_vel = self.checkAngularLimitVelocity(
                    self.target_angular_vel + ANG_VEL_STEP_SIZE
                )
            elif key == "d":
                self.target_angular_vel = self.checkAngularLimitVelocity(
                    self.target_angular_vel - ANG_VEL_STEP_SIZE
                )
            elif key == " " or key == "s":
                self.target_linear_vel = 0.0
                self.target_angular_vel = 0.0
            elif key == "\x03":
                self.destroy_timer(self.timer)
                return

            self.status += 1

            if self.status == 20:
                self.display()
                self.status = 0

        except:
            self.get_logger().error(e)
            self.destroy_timer(self.timer)
            return

        finally:
            cmd = VelAngle()
            cmd.vel = self.target_linear_vel
            if cmd.vel < 0:
                # With how the cart treats negative velocities, small numbers mean faster braking, so we invert it
                cmd.vel = -MAX_LIN_VEL - cmd.vel
            cmd.angle = self.target_angular_vel
            self.nav_pub.publish(cmd)


def main():
    """The main method that actually handles spinning up the node."""

    rclpy.init()
    node = Teleop()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
