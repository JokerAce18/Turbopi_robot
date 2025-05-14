# mecanum_drive_controller_node.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class MecanumDriveController(Node):
    def __init__(self):
        super().__init__('mecanum_drive_controller')

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)

        self.pub_fl = self.create_publisher(Float64, '/front_left_wheel_joint/command', 10)
        self.pub_fr = self.create_publisher(Float64, '/front_right_wheel_joint/command', 10)
        self.pub_rl = self.create_publisher(Float64, '/rear_left_wheel_joint/command', 10)
        self.pub_rr = self.create_publisher(Float64, '/rear_right_wheel_joint/command', 10)

        self.wheel_radius = 0.0325
        self.L = 0.0935
        self.W = 0.0695

    def cmd_vel_callback(self, msg):
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        r = self.wheel_radius
        L = self.L
        W = self.W

        fl = (1/r) * (vx - vy - (L + W) * wz)
        fr = (1/r) * (vx + vy + (L + W) * wz)
        rl = (1/r) * (vx + vy - (L + W) * wz)
        rr = (1/r) * (vx - vy + (L + W) * wz)

        self.pub_fl.publish(Float64(data=fl))
        self.pub_fr.publish(Float64(data=fr))
        self.pub_rl.publish(Float64(data=rl))
        self.pub_rr.publish(Float64(data=rr))

def main(args=None):
    rclpy.init(args=args)
    node = MecanumDriveController()
    rclpy.spin(node)
    rclpy.shutdown()
