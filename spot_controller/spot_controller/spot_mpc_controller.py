#!/usr/bin/python3
# Import necessary modules

import casadi as ca
import io
import sys
import warnings
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

warnings.filterwarnings("ignore", category=UserWarning, module="do_mpc.*")
import do_mpc


# Dummy stdout to silence IPOPT
class DummyFile(io.StringIO):
    def write(self, txt):
        pass


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
        self._dt = 0.3 # seconds
        self._progressed_time = 0.0 # seconds

        # Create a timer to call a function repeatedly over the time period
        self._timer = self.create_timer(self._dt, self.cmdloop_callback)

        self._cur_pos = np.array([0, 0])
        self._yaw = 0.0

        self._des_pos = np.array([0.0, 0.0])
        self._des_yaw = np.pi/4

        self._r_obs_1 = 0.0
        self._obsPos_1 = np.array([0.0, 0.0])
        self._r_obs_2 = 0.0
        self._obsPos_2 = np.array([0.0, 0.0])

        self._k_pos = 0.5
        self._k_yaw = 0.8

        self._max_vel_x = 0.3
        self._max_vel_y = 0.2
        self._max_vel_yaw = 0.3


        self._odom_flag = False

        self._pred_hor = 10
        self._step_time = 0.1

        self._tvp = None

        self._mpc = self.create_model()
        self._mpc.setup()


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

        if la.norm(posErr) < 0.1 or not self._odom_flag:
            goal_msg.linear.x = 0.0
            goal_msg.linear.y = 0.0

        else:

            # Write the steps for getting the control inputs from MPC.
            # Hint: You must call the updateStep() method here.


            R = np.array([[np.cos(self._yaw), np.sin(self._yaw)], [-np.sin(self._yaw), np.cos(self._yaw)]])
            u_body = R@u_2

            goal_msg.linear.x = np.clip(u_body[0], -self._max_vel_x, self._max_vel_x)
            goal_msg.linear.y = np.clip(u_body[1], -self._max_vel_y, self._max_vel_y)

        if np.abs(yawErr) < 0.1:
            goal_msg.angular.z = 0.0
        else:
            u_yaw = self._k_yaw * yawErr
            goal_msg.angular.z = np.clip(u_yaw, -self._max_vel_yaw, self._max_vel_yaw)

        self._command_pub.publish(goal_msg)
        rclpy.logging.get_logger('spot_controller').info(f'Command: X: {goal_msg.linear.x:.2f}, Y: {goal_msg.linear.y:.2f}, Yaw: {goal_msg.angular.z:.2f}')

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


    def create_model(self):
        model_type = 'continuous'
        sys_model = do_mpc.model.Model(model_type)

        # Write the code required to create the model for the MPC below this.

        return mpc

    def updateStep(self):
        # Silence IPOPT output
        sys.stdout = DummyFile()
        sys.stderr = DummyFile()

        # Write the MPC update below this.


        sys.stdout = sys.__stdout__
        sys.stderr = sys.__stderr__
        return u.flatten()


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