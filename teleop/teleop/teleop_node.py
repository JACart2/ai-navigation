import rclpy
import sys, select, os
import msvcrt, time
import tty, termios
from motor_control_interface.msg import VelAnglePlanned

MAX_LIN_VEL = 0.26
MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

msg = """
Control Your Golf Cart!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity (Burger : ~ 0.22, Waffle and Waffle Pi : ~ 0.26)
a/d : increase/decrease angular velocity (Burger : ~ 2.84, Waffle and Waffle Pi : ~ 1.82)

space key, s : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""


class Teleop(rclpy.node.Node):
    def __init__(self):
        super().__init__("teleop")

        self.settings = termios.tcgetattr(sys.stdin)

        self.nav_pub = self.create_publisher(VelAnglePlanned, "/nav_cmd", 10)
        self.timer = self.create_timer(10, self.timer_callback)

        self.status = 0
        self.target_linear_vel = 0.0
        self.target_angular_vel = 0.0
        self.control_linear_vel = 0.0
        self.control_angular_vel = 0.0

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ""

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def vels(self, target_linear_vel, target_angular_vel):
        return "currently:\tlinear vel %s\t angular vel %s " % (
            target_linear_vel,
            target_angular_vel,
        )

    def makeSimpleProfile(self, output, input, slop):
        if input > output:
            output = min(input, output + slop)
        elif input < output:
            output = max(input, output - slop)
        else:
            output = input

        return output

    def constrain(self, input, low, high):
        if input < low:
            input = low
        elif input > high:
            input = high
        else:
            input = input

        return input

    def checkLinearLimitVelocity(self, vel):
        vel = self.constrain(vel, -MAX_LIN_VEL, MAX_LIN_VEL)
        return vel

    def checkAngularLimitVelocity(self, vel):
        vel = self.constrain(vel, -MAX_ANG_VEL, MAX_ANG_VEL)
        return vel

    def timer_callback(self):
        try:
            print(msg)
            key = self.getKey()
            if key == "w":
                target_linear_vel = self.checkLinearLimitVelocity(
                    target_linear_vel + LIN_VEL_STEP_SIZE
                )
                status = status + 1
                print(self.vels(target_linear_vel, target_angular_vel))
            elif key == "x":
                target_linear_vel = self.checkLinearLimitVelocity(
                    target_linear_vel - LIN_VEL_STEP_SIZE
                )
                status = status + 1
                print(self.vels(target_linear_vel, target_angular_vel))
            elif key == "a":
                target_angular_vel = self.checkAngularLimitVelocity(
                    target_angular_vel + ANG_VEL_STEP_SIZE
                )
                status = status + 1
                print(self.vels(target_linear_vel, target_angular_vel))
            elif key == "d":
                target_angular_vel = self.checkAngularLimitVelocity(
                    target_angular_vel - ANG_VEL_STEP_SIZE
                )
                status = status + 1
                print(self.vels(target_linear_vel, target_angular_vel))
            elif key == " " or key == "s":
                target_linear_vel = 0.0
                control_linear_vel = 0.0
                target_angular_vel = 0.0
                control_angular_vel = 0.0
                print(self.vels(target_linear_vel, target_angular_vel))
            else:
                if key == "\x03":
                    self.destroy_timer(self.timer)
                    return

            if status == 20:
                print(msg)
                status = 0

            cmd = VelAnglePlanned()

            control_linear_vel = self.makeSimpleProfile(
                control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE / 2.0)
            )

            control_angular_vel = self.makeSimpleProfile(
                control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE / 2.0)
            )

            cmd.vel_planned = control_linear_vel
            cmd.angle_planned = control_angular_vel
            self.nav_pub.publish(cmd)
        except:
            print(e)

        finally:
            cmd = VelAnglePlanned()
            cmd.vel_planned = 0.0
            cmd.angle_planned = 0.0
            self.nav_pub.publish(cmd)


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
