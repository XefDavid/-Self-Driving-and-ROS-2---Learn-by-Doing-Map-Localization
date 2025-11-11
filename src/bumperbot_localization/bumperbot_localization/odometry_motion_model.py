#!/usr/bin/env python3

from math import sin, cos, atan2, sqrt, fabs, pi
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import random
import time

def normalize(z):
    return atan2(sin(z), cos(z))

def angle_diff(a, b):
    a = normalize(a)
    b = normalize(b)
    d1 = a - b
    d2 = 2 * pi - fabs(d1)
    if d1 > 0:
        d2 *= -1.0
    if fabs(d1) < fabs(d2):
        return d1
    else:
        return d2

class OdometryMotionModel(Node):
    def __init__(self):
        super().__init__('odometry_motion_model')

        # Última odometría conocida
        self.last_odom_x = 0.0
        self.last_odom_y = 0.0
        self.last_odom_theta = 0.0
        self.is_first_odom = True

        # parámetros
        self.declare_parameter('alpha1', 0.1)
        self.declare_parameter('alpha2', 0.1)
        self.declare_parameter('alpha3', 0.1)
        self.declare_parameter('alpha4', 0.1)
        self.declare_parameter('nr_samples', 300)

        self.alpha1 = float(self.get_parameter('alpha1').get_parameter_value().double_value)
        self.alpha2 = float(self.get_parameter('alpha2').get_parameter_value().double_value)
        self.alpha3 = float(self.get_parameter('alpha3').get_parameter_value().double_value)
        self.alpha4 = float(self.get_parameter('alpha4').get_parameter_value().double_value)
        self.nr_samples = int(self.get_parameter('nr_samples').get_parameter_value().double_value)

        if self.nr_samples >= 0:
            self.samples = PoseArray()
            self.samples.header.frame_id = 'odom'  # se sobrescribirá en el primer mensaje real
            self.samples.poses = [Pose() for _ in range(self.nr_samples)]
        else:
            self.get_logger().fatal(f"Invalid number of sample request {self.nr_samples} exit")
            return

        # sub y pub
        self.odom_sub = self.create_subscription(Odometry, 'bumperbot_controller/odom', self.odom_callback, 10)
        self.pose_array_pub = self.create_publisher(PoseArray, 'Odometry_motion_model/samples', 10)

    def odom_callback(self, odom: Odometry):
        q = [
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w
        ]
        _, _, yaw = euler_from_quaternion(q)

        if self.is_first_odom:
            self.samples.header.frame_id = odom.header.frame_id
            self.last_odom_x = odom.pose.pose.position.x
            self.last_odom_y = odom.pose.pose.position.y
            self.last_odom_theta = yaw
            self.is_first_odom = False
            return

        odom_x_increment = odom.pose.pose.position.x - self.last_odom_x
        odom_y_increment = odom.pose.pose.position.y - self.last_odom_y
        odom_theta_increment = angle_diff(yaw, self.last_odom_theta)

        if sqrt(odom_x_increment**2 + odom_y_increment**2) < 0.01:
            delta_rot1 = 0.0
        else:
            delta_rot1 = angle_diff(atan2(odom_y_increment, odom_x_increment), yaw)

        delta_trans = sqrt(odom_x_increment**2 + odom_y_increment**2)
        delta_rot2 = angle_diff(odom_theta_increment, delta_rot1)

        rot1_variance = self.alpha1 * fabs(delta_rot1) + self.alpha2 * delta_trans
        trans_variance = self.alpha3 * delta_trans + self.alpha4 * (fabs(delta_rot1) + fabs(delta_rot2))
        rot2_variance = self.alpha1 * fabs(delta_rot2) + self.alpha2 * delta_trans

        random.seed(time.time())

        for sample in self.samples.poses:
            rot1_noise = random.gauss(0.0, rot1_variance)
            rot2_noise = random.gauss(0.0, rot2_variance)
            trans_noise = random.gauss(0.0, trans_variance)

            delta_rot1_draw = angle_diff(delta_rot1, rot1_noise)
            delta_rot2_draw = angle_diff(delta_rot2, rot2_noise)
            delta_trans_draw = delta_trans - trans_noise

            sample_q = [
                sample.orientation.x,
                sample.orientation.y,
                sample.orientation.z,
                sample.orientation.w
            ]
            _, _, sample_yaw = euler_from_quaternion(sample_q)

            sample.position.x += delta_trans_draw * cos(sample_yaw + delta_rot1_draw)
            sample.position.y += delta_trans_draw * sin(sample_yaw + delta_rot1_draw)

            q_new = quaternion_from_euler(0.0, 0.0, sample_yaw + delta_rot1_draw + delta_rot2_draw)
            sample.orientation.x, sample.orientation.y, sample.orientation.z, sample.orientation.w = q_new

        self.last_odom_x = odom.pose.pose.position.x
        self.last_odom_y = odom.pose.pose.position.y
        self.last_odom_theta = yaw

        self.samples.header.stamp = odom.header.stamp
        self.pose_array_pub.publish(self.samples)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryMotionModel()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
