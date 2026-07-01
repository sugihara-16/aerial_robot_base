#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026, DRAGON Laboratory, The University of Tokyo
import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

from aerial_robot_msgs.msg import FlightNav

instruction_text = """
Instruction:

---------------------------

r:  Arm motors (before takeoff)
t:  Takeoff
l:  Land
f:  Force landing
h:  Halt (force stop motors)

     q           w           e           [
(turn left)  (forward)  (turn right)  (move up)

     a           s           d           ]
(move left)  (backward) (move right) (move down)

     u           i
(+roll)     (+pitch)

     j           k
(-roll)     (-pitch)

x:  send task-start command

Avoid caps-lock.
CTRL+c to quit.

---------------------------
"""


def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def printMsg(msg: str, width: int = 50):
    print(msg.ljust(width) + "\r", end="", flush=True)


class KeyboardCommandNode(Node):
    def __init__(self):
        super().__init__("keyboard_command")

        # ------------------------------------------------------------------
        # Define & get parameters from CLI or default values
        # ------------------------------------------------------------------
        self.declare_parameter("robot_ns", "")
        self.declare_parameter("xy_vel", 0.2)
        self.declare_parameter("z_vel", 0.2)
        self.declare_parameter("yaw_vel", 0.2)
        self.declare_parameter("rp_vel", 0.1)

        robot_ns = self.get_parameter("robot_ns").get_parameter_value().string_value
        self.xy_vel = self.get_parameter("xy_vel").get_parameter_value().double_value
        self.z_vel = self.get_parameter("z_vel").get_parameter_value().double_value
        self.yaw_vel = self.get_parameter("yaw_vel").get_parameter_value().double_value
        self.rp_vel = self.get_parameter("rp_vel").get_parameter_value().double_value

        # If robot_ns is not specified as CLI argument, parse the namespace from core node name
        if not robot_ns:
            robot_ns = self._detect_namespace()

        ns = (robot_ns.rstrip("/") + "/teleop_command") if robot_ns else "teleop_command"
        nav_base = (robot_ns.rstrip("/") + "/uav/nav") if robot_ns else "uav/nav"

        self.land_pub = self.create_publisher(Empty, ns + "/land", 1)
        self.halt_pub = self.create_publisher(Empty, ns + "/halt", 1)
        self.start_pub = self.create_publisher(Empty, ns + "/start", 1)
        self.takeoff_pub = self.create_publisher(Empty, ns + "/takeoff", 1)
        self.force_landing_pub = self.create_publisher(Empty, ns + "/force_landing", 1)
        self.nav_pub = self.create_publisher(FlightNav, nav_base, 1)
        self.motion_start_pub = self.create_publisher(Empty, "task_start", 1)

    def run(self):
        print(instruction_text)
        while rclpy.ok():
            key = getKey()
            msg_text = ""

            if key == "l":
                self.land_pub.publish(Empty())
                msg_text = "Sent land command!"
            elif key == "r":
                self.start_pub.publish(Empty())
                msg_text = "Sent motor-arming command!"
            elif key == "h":
                self.halt_pub.publish(Empty())
                msg_text = "Sent halt command!"
            elif key == "f":
                self.force_landing_pub.publish(Empty())
                msg_text = "Sent force landing command!"
            elif key == "t":
                self.takeoff_pub.publish(Empty())
                msg_text = "Sent takeoff command!"
            elif key == "x":
                self.motion_start_pub.publish(Empty())
                msg_text = "Sent task-start command"
            elif key in ("w", "s", "a", "d", "q", "e", "[", "]", "u", "j", "i", "k"):
                nav_msg = FlightNav()
                nav_msg.control_frame = FlightNav.WORLD_FRAME
                nav_msg.target = FlightNav.COG

                if key == "w":
                    nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
                    nav_msg.target_vel_x = self.xy_vel
                    msg_text = "Sent +x vel command"
                elif key == "s":
                    nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
                    nav_msg.target_vel_x = -self.xy_vel
                    msg_text = "Sent -x vel command"
                elif key == "a":
                    nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
                    nav_msg.target_vel_y = self.xy_vel
                    msg_text = "Sent +y vel command"
                elif key == "d":
                    nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
                    nav_msg.target_vel_y = -self.xy_vel
                    msg_text = "Sent -y vel command"
                elif key == "q":
                    nav_msg.yaw_nav_mode = FlightNav.VEL_MODE
                    nav_msg.target_omega_z = self.yaw_vel
                    msg_text = "Sent +yaw vel command"
                elif key == "e":
                    nav_msg.yaw_nav_mode = FlightNav.VEL_MODE
                    nav_msg.target_omega_z = -self.yaw_vel
                    msg_text = "Sent -yaw vel command"
                elif key == "[":
                    nav_msg.pos_z_nav_mode = FlightNav.VEL_MODE
                    nav_msg.target_vel_z = self.z_vel
                    msg_text = "Sent +z vel command"
                elif key == "]":
                    nav_msg.pos_z_nav_mode = FlightNav.VEL_MODE
                    nav_msg.target_vel_z = -self.z_vel
                    msg_text = "Sent -z vel command"
                elif key == "u":
                    nav_msg.roll_nav_mode = FlightNav.VEL_MODE
                    nav_msg.target_omega_x = self.rp_vel
                    msg_text = "Sent +roll vel command"
                elif key == "j":
                    nav_msg.roll_nav_mode = FlightNav.VEL_MODE
                    nav_msg.target_omega_x = -self.rp_vel
                    msg_text = "Sent -roll vel command"
                elif key == "i":
                    nav_msg.pitch_nav_mode = FlightNav.VEL_MODE
                    nav_msg.target_omega_y = self.rp_vel
                    msg_text = "Sent +pitch vel command"
                elif key == "k":
                    nav_msg.pitch_nav_mode = FlightNav.VEL_MODE
                    nav_msg.target_omega_y = -self.rp_vel
                    msg_text = "Sent -pitch vel command"

                self.nav_pub.publish(nav_msg)

            elif key == "\x03":  # Ctrl+C
                break

            printMsg(msg_text)

            # Yield to the ROS 2 executor briefly (callbacks, etc.)
            rclpy.spin_once(self, timeout_sec=0.001)

    def _detect_namespace(self) -> str:
        max_tries = 100
        for _ in range(max_tries):
            topic_names_and_types = self.get_topic_names_and_types()
            candidates = [name for name, _ in topic_names_and_types if name.endswith("/teleop_command/start")]
            if len(candidates) == 1:
                return candidates[0].rsplit("/teleop_command", 1)[0]

            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().warn(
            "robot_ns is empty! Try setting the namespace as CLI argument. " "Publishing on global topics..."
        )
        return ""


def main():
    global settings
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = KeyboardCommandNode()
    try:
        node.run()
    except Exception as e:
        print(repr(e))
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
