#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, TwistStamped
import time

class XboxTeleop(Node):
    def __init__(self):
        super().__init__('xbox_teleop')

        # --- Parameters ---
        # Axis Indices (Standard Xbox Controller on Linux)
        self.AXIS_LEFT_H = 0
        self.AXIS_LEFT_V = 1
        self.AXIS_RIGHT_H = 3
        self.AXIS_RIGHT_V = 4
        self.TRIGGER_LEFT = 2
        self.TRIGGER_RIGHT = 5
        
        # Button Indices
        self.BTN_A = 0
        self.BTN_B = 1
        self.BTN_X = 2
        self.BTN_Y = 3
        self.BTN_LB = 4
        self.BTN_RB = 5

        # Configuration
        self.declare_parameter('base_frame', 'xarm_link_base') # Frame for arm commands
        self.declare_parameter('scale_linear_base', 0.5)
        self.declare_parameter('scale_angular_base', 1.0)
        self.declare_parameter('scale_linear_arm', 0.2)  # Slower for precision
        self.declare_parameter('scale_angular_arm', 0.5)

        # State
        self.mode = 'BASE' # 'BASE' or 'ARM'
        
        # Publishers
        # 1. Base Control (Standard Diff Drive)
        self.base_pub = self.create_publisher(Twist, '/platform_velocity_controller/cmd_vel_unstamped', 10)
        
        # 2. Arm Control (MoveIt Servo)
        # Publishes Cartesian velocity requests to the Servo server
        self.arm_pub = self.create_publisher(TwistStamped, '/servo_server/delta_twist_cmds', 10)
        
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        
        self.get_logger().info("Xbox Teleop Ready.")
        self.get_logger().info("  LB: Switch to BASE mode (Left Stick to drive)")
        self.get_logger().info("  RB: Switch to ARM mode (Left Stick XY, Y/X for Z, Right Stick Rot)")

    def joy_callback(self, msg):
        # --- Mode Switching ---
        if msg.buttons[self.BTN_LB]:
            if self.mode != 'BASE':
                self.mode = 'BASE'
                self.get_logger().info("MODE: BASE CONTROL (Left Stick to Drive)")
        elif msg.buttons[self.BTN_RB]:
            if self.mode != 'ARM':
                self.mode = 'ARM'
                self.get_logger().info("MODE: ARM CONTROL (Left Stick=XY, Y/X=Z, Right Stick=Rot)")

        # --- Deadman Switch (Safety) ---
        # We generally require a button to be held to move anything (usually 'A' or a trigger)
        # For this example, let's say holding 'Left Trigger' enables movement to prevent accidents
        # Adjust threshold (-1.0 is released, 1.0 is pressed usually, or 0 to -1 depending on driver)
        # Simple check: If A button is NOT held, send zero commands and return
        # (You can remove this if you prefer continuous enabling)
        # if not msg.buttons[self.BTN_A]:
        #     self.stop_all()
        #     return

        if self.mode == 'BASE':
            self.drive_base(msg)
        else:
            self.drive_arm(msg)

    def drive_base(self, msg):
        twist = Twist()
        scale_lin = self.get_parameter('scale_linear_base').value
        scale_ang = self.get_parameter('scale_angular_base').value

        # Left Joystick for Base Driving
        twist.linear.x = msg.axes[self.AXIS_LEFT_V] * scale_lin
        twist.angular.z = msg.axes[self.AXIS_LEFT_H] * scale_ang

        self.base_pub.publish(twist)
        # Stop arm just in case
        self.arm_pub.publish(TwistStamped())

    def drive_arm(self, msg):
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = self.get_parameter('base_frame').value

        lin_scale = self.get_parameter('scale_linear_arm').value
        ang_scale = self.get_parameter('scale_angular_arm').value

        # --- Linear (Left Stick + Buttons) ---
        # X/Y controlled by Left Joystick
        twist_stamped.twist.linear.y = msg.axes[self.AXIS_LEFT_H] * lin_scale
        twist_stamped.twist.linear.x = msg.axes[self.AXIS_LEFT_V] * lin_scale
        
        # Z controlled by Buttons
        if msg.buttons[self.BTN_Y]: # UP
            twist_stamped.twist.linear.z = 1.0 * lin_scale
        elif msg.buttons[self.BTN_X]: # DOWN
            twist_stamped.twist.linear.z = -1.0 * lin_scale

        # --- Angular (Right Stick + Buttons) ---
        # Roll/Pitch controlled by Right Joystick
        twist_stamped.twist.angular.y = msg.axes[self.AXIS_RIGHT_V] * ang_scale # Pitch
        twist_stamped.twist.angular.x = msg.axes[self.AXIS_RIGHT_H] * ang_scale # Roll

        # Yaw controlled by A/B Buttons
        if msg.buttons[self.BTN_B]: # Yaw Left (CCW)
            twist_stamped.twist.angular.z = 1.0 * ang_scale
        elif msg.buttons[self.BTN_A]: # Yaw Right (CW)
            twist_stamped.twist.angular.z = -1.0 * ang_scale

        self.arm_pub.publish(twist_stamped)
        # Stop base
        self.base_pub.publish(Twist())

    def stop_all(self):
        self.base_pub.publish(Twist())
        self.arm_pub.publish(TwistStamped())

def main(args=None):
    rclpy.init(args=args)
    node = XboxTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()