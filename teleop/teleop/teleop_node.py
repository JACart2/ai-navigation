import rclpy
import sys, select, os
import msvcrt, time
import tty, termios
from motor_control_interface.msg import VelAnglePlanned

MAX_LIN_VEL = 30
MAX_ANG_VEL = 30

LIN_VEL_STEP_SIZE = 0.5
ANG_VEL_STEP_SIZE = 0.5

msg = """
Control Your Golf Cart!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity 
a/d : increase/decrease angular velocity 

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
                    
            print(self.vels(self.target_linear_vel, self.target_angular_vel))
            self.status += 1
                
            if status == 20:
                print(msg)
                status = 0
                    
        except Exception as e:
            print(e)

        finally:
            cmd = VelAnglePlanned()
            cmd.vel_planned = self.target_linear_vel
            if (cmd.vel_planned < 0):
                cmd.vel_planned = -MAX_LIN_VEL - cmd.vel_planned
            cmd.angle_planned = self.target_angular_vel
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
