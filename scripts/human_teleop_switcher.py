#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

import sys
import select
import termios
import tty


class HumanTeleopSwitcher(Node):

    def __init__(self):
        super().__init__('human_teleop_switcher')


        # Publisher
        self.human1_pub = self.create_publisher(
            Twist,
            '/human1/cmd_vel',
            10
        )

        self.human2_pub = self.create_publisher(
            Twist,
            '/human2/cmd_vel',
            10
        )


        # 操作対象
        self.target = "human1"


        # 速度
        self.linear_speed = 0.5
        self.angular_speed = 1.0

        self.speed_step = 1.1

        self.max_linear_speed = 2.0
        self.max_angular_speed = 5.0

        self.min_linear_speed = 0.1
        self.min_angular_speed = 0.1


        # terminal保存
        self.settings = termios.tcgetattr(sys.stdin)


        self.print_help()



    def print_help(self):

        msg = """

---------------------------
Human Actor Teleop Switcher
---------------------------

Moving around:

   u    i    o
   j    k    l
   m    ,    .

anything else : stop


Actor switching:

   1 : human1
   2 : human2


Speed control:

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%


CTRL-C to quit

currently:
    target : human1
    speed  : {:.2f}
    turn   : {:.2f}

---------------------------

""".format(
            self.linear_speed,
            self.angular_speed
        )


        print(msg)



    def publish_twist(self, msg):

        if self.target == "human1":

            self.human1_pub.publish(msg)


        elif self.target == "human2":

            self.human2_pub.publish(msg)



    def keyboard_loop(self):

        key = self.get_key()


        # =====================
        # Actor切替
        # =====================

        if key == '1':

            self.target = "human1"

            print(
                "\nCurrent target : human1"
            )

            return


        if key == '2':

            self.target = "human2"

            print(
                "\nCurrent target : human2"
            )

            return



        # =====================
        # 速度変更
        # =====================

        if key == 'q':

            self.linear_speed *= self.speed_step
            self.angular_speed *= self.speed_step


            self.linear_speed = min(
                self.linear_speed,
                self.max_linear_speed
            )

            self.angular_speed = min(
                self.angular_speed,
                self.max_angular_speed
            )


            self.print_status()

            return



        if key == 'z':

            self.linear_speed /= self.speed_step
            self.angular_speed /= self.speed_step


            self.linear_speed = max(
                self.linear_speed,
                self.min_linear_speed
            )

            self.angular_speed = max(
                self.angular_speed,
                self.min_angular_speed
            )


            self.print_status()

            return



        if key == 'w':

            self.linear_speed *= self.speed_step

            self.linear_speed = min(
                self.linear_speed,
                self.max_linear_speed
            )

            self.print_status()

            return



        if key == 'x':

            self.linear_speed /= self.speed_step

            self.linear_speed = max(
                self.linear_speed,
                self.min_linear_speed
            )

            self.print_status()

            return



        if key == 'e':

            self.angular_speed *= self.speed_step

            self.angular_speed = min(
                self.angular_speed,
                self.max_angular_speed
            )

            self.print_status()

            return



        if key == 'c':

            self.angular_speed /= self.speed_step

            self.angular_speed = max(
                self.angular_speed,
                self.min_angular_speed
            )

            self.print_status()

            return



        # =====================
        # Twist生成
        # =====================

        twist = Twist()


        if key == 'i':

            twist.linear.x = self.linear_speed


        elif key == ',':

            twist.linear.x = -self.linear_speed


        elif key == 'j':

            twist.angular.z = self.angular_speed


        elif key == 'l':

            twist.angular.z = -self.angular_speed


        elif key == 'u':

            twist.linear.x = self.linear_speed
            twist.angular.z = self.angular_speed


        elif key == 'o':

            twist.linear.x = self.linear_speed
            twist.angular.z = -self.angular_speed


        elif key == 'm':

            twist.linear.x = -self.linear_speed
            twist.angular.z = self.angular_speed


        elif key == '.':

            twist.linear.x = -self.linear_speed
            twist.angular.z = -self.angular_speed


        elif key == 'k':

            twist.linear.x = 0.0
            twist.angular.z = 0.0


        else:

            return


        self.publish_twist(twist)



    def print_status(self):

        print(
            "\ncurrently:"
            f" target {self.target}"
            f"  speed {self.linear_speed:.2f}"
            f"  turn {self.angular_speed:.2f}"
        )



    def get_key(self):

        tty.setraw(
            sys.stdin.fileno()
        )


        rlist, _, _ = select.select(
            [sys.stdin],
            [],
            [],
            0.1
        )


        if rlist:

            key = sys.stdin.read(1)

        else:

            key = ''


        termios.tcsetattr(
            sys.stdin,
            termios.TCSADRAIN,
            self.settings
        )


        return key




def main(args=None):

    rclpy.init(args=args)

    node = HumanTeleopSwitcher()


    try:

        while rclpy.ok():

            node.keyboard_loop()

            rclpy.spin_once(
                node,
                timeout_sec=0.01
            )


    except KeyboardInterrupt:

        pass


    finally:

        termios.tcsetattr(
            sys.stdin,
            termios.TCSADRAIN,
            node.settings
        )

        node.destroy_node()

        rclpy.shutdown()



if __name__ == '__main__':

    main()