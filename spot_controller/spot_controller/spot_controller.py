#!/usr/bin/python3
# Import necessary modules
import os
import sys
import rclpy
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
import numpy as np
import numpy.linalg as la

# Define a class for the JointCommander node
class Controller(Node):
    def __init__(self, namespace=''):
        # Initialize the node with the name 'four_joint_commander'
        super().__init__('spot_controller')
        
        # Define the QoS profile for the publisher
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        # Create a publisher for the position controller commands
        self._command_pub = self.create_publisher(Twist, '/cmd_vel', qos_profile)
        self._odom_sub = self.create_subscription(Odometry, '/spot/odometry', self.odom_callback, qos_profile)
        self._pose_sub = self.create_subscription(Pose, '/new_pose', self.sp_callback, qos_profile)

        # Time period
        self._dt = 0.2 # seconds

        # Create a timer to call a function repeatedly over the time period
        self._timer = self.create_timer(self._dt, self.cmdloop_callback)

        self._cur_pos = np.array([0, 0])
        self._yaw = 0.0

        self._des_pos = np.array([0.0, 0.0])
        self._des_yaw = 0.0

        self._k_pos = 0.5
        self._k_yaw = 0.8

        self._max_vel_x = 1.0
        self._max_vel_y = 0.3
        self._max_vel_yaw = 0.3


        self._odom_flag = False


    def cmdloop_callback(self):
        """
        Timed callback function to publish the joint angles for the end-effectors to track circles
        """

        # Create a goal message
        goal_msg = Twist()

        posErr = self._des_pos - self._cur_pos
        yawErr = self._des_yaw - self._yaw

        if yawErr > np.pi:
            yawErr = -(2*np.pi - np.abs(yawErr))*np.sign(yawErr)

        # print(f'Command: {posErr[0]:.2f}: {posErr[1]:.2f}')
        if la.norm(posErr) < 0.1 or not self._odom_flag:
            goal_msg.linear.x = 0.0
            goal_msg.linear.y = 0.0
            print(f'Goal Reached.')


        else: 
            u_2 = self._k_pos * posErr

            R = np.array([[np.cos(self._yaw), np.sin(self._yaw)], [-np.sin(self._yaw), np.cos(self._yaw)]])
            u_body = R@u_2

            goal_msg.linear.x = np.clip(u_body[0], -self._max_vel_x, self._max_vel_x)
            goal_msg.linear.y = np.clip(u_body[1], -self._max_vel_y, self._max_vel_y)

        if np.abs(yawErr) < 0.05:
            goal_msg.angular.z = 0.0
        else:
            u_yaw = self._k_yaw * yawErr
            goal_msg.angular.z = np.clip(u_yaw, -self._max_vel_yaw, self._max_vel_yaw)

        self._command_pub.publish(goal_msg)
        print(f'Command: X: {goal_msg.linear.x:.2f}, Y: {goal_msg.linear.y:.2f}, Yaw: {goal_msg.angular.z}')

    def odom_callback(self, msg):
        self._cur_pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        q = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self._yaw = np.arctan2(2*(q[0]*q[1] + q[2]*q[3]), (1 - 2*(q[2]*q[2] + q[1]*q[1])))
        self._odom_flag = True

    def sp_callback(self, msg):
        self._des_pos = np.array([msg.position.x, msg.position.y])
        q = np.array([msg.orientation.x, msg.orientation.y,
                        msg.orientation.z, msg.orientation.w])
        self._des_yaw = np.arctan2(2*(q[0]*q[1] + q[2]*q[3]), (1 - 2*(q[2]*q[2] + q[1]*q[1])))

def main(args=None):
    """
    Main function to initialize the ROS 2 node and start the controller.
    """
    # Initialize the ROS 2 Python client library
    rclpy.init()

    # Create an instance of the JointCommander
    controller = Controller()

    try:
        # Spin the node to keep it active
        rclpy.spin(controller)
    except SystemExit:
        # Log a message when shutting down
        rclpy.logging.get_logger('spot_controller').info('Shutting down the spot_controller')
    
    # Destroy the node explicitly
    controller.destroy_node()

    # Shutdown the ROS 2 Python client library
    rclpy.shutdown()

if __name__ == '__main__':
    main()