#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from planner import *

from std_srvs.srv import SetBool


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(SetBool, '/busbot/custom_switch')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetBool.Request()

    def send_request(self, status):
        self.req.data = status
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


class Controller_Node(Node):  # Define the Command Velocity Publisher Node

    def __init__(self):  # Init Function

        super().__init__('controller_node')  # Naming the Node
        # Confirmation that the Node is Spun Properly
        self.get_logger().info(f'Controller node on.')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)

        # Create a publisher to the wheel velocoty controller
        self.wheel_velocities_pub = self.create_publisher(
            Float64MultiArray, '/velocity_controller/commands', 10)
        # Create a publisher to the joint position controller
        self.joint_position_pub = self.create_publisher(
            Float64MultiArray, '/position_controller/commands', 10)

        # Create a subscriber to the 'imu_plugin/out' topic
        self.imu_sub = self.create_subscription(
            Imu, 'imu_plugin/out', self.imu_callback, qos_profile)
        self.imu_sub  # Prevent unused variable warning

        timer_period = 0.1  # Pub Frequency
        self.timer = self.create_timer(
            timer_period, self.timer_callback)  # Initialize Timer Callback

        # Initialize Counters Used for Publishing Execution
        self.counter = 0
        self.t1_ctr = 0
        self.t2_ctr = 0
        self.t3_ctr = 0
        self.t4_ctr = 0

        # Initialize Booleans Used for Switch-Case Execution
        self.driving = True
        self.traj1 = False
        self.traj2 = False
        self.traj3 = False
        self.traj4 = False

    def imu_callback(self, msg):  # Define the Callback Function
        # Grab the Time of record.
        imu_timestamp = msg.header.stamp.sec + \
            (msg.header.stamp.nanosec)/(1000000000)
        # Grab the robot's current yaw (orientation).
        current_heading = msg.orientation.z

        self.timestamp = imu_timestamp  # Update the node's timestamp.
        # Update the robot's current heading.
        self.current_heading = current_heading

    def timer_callback(self):  # Defining the Callback Function
        # Initialize Plans from Planner

        # Initialize Messages
        wheel_velocities = Float64MultiArray()
        joint_positions = Float64MultiArray()
        linear_vel = 1.0

        if self.driving == True:
            self.counter += 1  # Increase Counter
            # Drive Forward
            if self.counter < 75:
                wheel_velocities.data = [-linear_vel,
                                         linear_vel, -linear_vel, linear_vel]
                joint_positions.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            else:
                # Stop
                wheel_velocities.data = [0.0, 0.0, 0.0, 0.0]
                joint_positions.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                # Switch to Trajectory 1 Execution
                self.driving = False
                self.traj1 = True

        elif self.traj1 == True:
            self.get_logger().info(f'Executing Trajectory 1!')
            # Form Large Array of Position Controller Angles
            n_traj1 = len(q1_array_step1)
            ja_t1 = np.column_stack(
                (q1_array_step1, q2_array_step1, q3_array_step1, q4_array_step1))
            jsa_traj1 = np.hstack((np.zeros((n_traj1, 2)), ja_t1))
            # Index proper message from array
            if self.t1_ctr < n_traj1:
                joint_positions.data = [float(jsa_traj1[self.t1_ctr, 0]),
                                        float(jsa_traj1[self.t1_ctr, 1]),
                                        float(jsa_traj1[self.t1_ctr, 2]),
                                        float(jsa_traj1[self.t1_ctr, 3]),
                                        float(jsa_traj1[self.t1_ctr, 4]),
                                        float(jsa_traj1[self.t1_ctr, 5])]
                wheel_velocities.data = [0.0, 0.0, 0.0, 0.0]
                self.t1_ctr += 1

            else:
                self.get_logger().info(f'Finish Trajectory 1!')
                wheel_velocities.data = [0.0, 0.0, 0.0, 0.0]
                # Switch to Trajectory 2 Execution
                self.traj1 = False
                self.traj2 = True

        elif self.traj2 == True:
            self.get_logger().info(f'Executing Trajectory 2!')
            # Form Large Array of Position Controller Angles
            n_traj2 = len(q1_array_step2)
            ja_t2 = np.column_stack(
                (q1_array_step2, q2_array_step2, q3_array_step2, q4_array_step2))
            jsa_traj2 = np.hstack((np.zeros((n_traj2, 2)), ja_t2))
            # Index proper message from array
            if self.t2_ctr < n_traj2:
                joint_positions.data = [float(jsa_traj2[self.t2_ctr, 0]),
                                        float(jsa_traj2[self.t2_ctr, 1]),
                                        float(jsa_traj2[self.t2_ctr, 2]),
                                        float(jsa_traj2[self.t2_ctr, 3]),
                                        float(jsa_traj2[self.t2_ctr, 4]),
                                        float(jsa_traj2[self.t2_ctr, 5])]
                wheel_velocities.data = [0.0, 0.0, 0.0, 0.0]
                self.t2_ctr += 1

            else:
                self.get_logger().info(f'Finish Trajectory 2!')
                wheel_velocities.data = [0.0, 0.0, 0.0, 0.0]
                joint_positions.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                # Switch to Trajectory 3 Execution
                self.traj2 = False
                self.traj3 = True

        elif self.traj3 == True:
            self.get_logger().info(f'Executing Trajectory 3!')
            # Form Large Array of Position Controller Angles
            n_traj3 = len(q1_array_step3)
            ja_t3 = np.column_stack(
                (q1_array_step3, q2_array_step3, q3_array_step3, q4_array_step3))
            jsa_traj3 = np.hstack((np.zeros((n_traj3, 2)), ja_t3))
            # Index proper message from array
            if self.t3_ctr < n_traj3:
                joint_positions.data = [float(jsa_traj3[self.t3_ctr, 0]),
                                        float(jsa_traj3[self.t3_ctr, 1]),
                                        float(jsa_traj3[self.t3_ctr, 2]),
                                        float(jsa_traj3[self.t3_ctr, 3]),
                                        float(jsa_traj3[self.t3_ctr, 4]),
                                        float(jsa_traj3[self.t3_ctr, 5])]
                wheel_velocities.data = [0.0, 0.0, 0.0, 0.0]
                self.t3_ctr += 1

            else:
                self.get_logger().info(f'Finish Trajectory 3!')
                wheel_velocities.data = [0.0, 0.0, 0.0, 0.0]
                # Switch to Trajectory 4 Execution
                self.traj3 = False
                self.traj4 = True

        elif self.traj4 == True:
            self.get_logger().info(f'Executing Trajectory 4!')
            # Form Large Array of Position Controller Angles
            n_traj4 = len(q1_array_step4)
            ja_t4 = np.column_stack(
                (q1_array_step4, q2_array_step4, q3_array_step4, q4_array_step4))
            jsa_traj4 = np.hstack((np.zeros((n_traj4, 2)), ja_t4))
            # Index proper message from array
            if self.t4_ctr < n_traj4:
                joint_positions.data = [float(jsa_traj4[self.t4_ctr, 0]),
                                        float(jsa_traj4[self.t4_ctr, 1]),
                                        float(jsa_traj4[self.t4_ctr, 2]),
                                        float(jsa_traj4[self.t4_ctr, 3]),
                                        float(jsa_traj4[self.t4_ctr, 4]),
                                        float(jsa_traj4[self.t4_ctr, 5])]
                wheel_velocities.data = [0.0, 0.0, 0.0, 0.0]
                self.t4_ctr += 1

            else:
                self.get_logger().info(f'Finish Trajectory 4!')
                wheel_velocities.data = [0.0, 0.0, 0.0, 0.0]
                self.traj4 = False
        # Keep robot at last position
        else:
            n_traj4 = len(q1_array_step4)
            ja_t4 = np.column_stack(
                (q1_array_step4, q2_array_step4, q3_array_step4, q4_array_step4))
            jsa_traj4 = np.hstack((np.zeros((n_traj4, 2)), ja_t4))
            wheel_velocities.data = [0.0, 0.0, 0.0, 0.0]
            joint_positions.data = [float(jsa_traj4[self.t4_ctr-1, 0]),
                                    float(jsa_traj4[self.t4_ctr-1, 1]),
                                    float(jsa_traj4[self.t4_ctr-1, 2]),
                                    float(jsa_traj4[self.t4_ctr-1, 3]),
                                    float(jsa_traj4[self.t4_ctr-1, 4]),
                                    float(jsa_traj4[self.t4_ctr-1, 5])]

        # Publish Messages
        self.wheel_velocities_pub.publish(wheel_velocities)
        self.joint_position_pub.publish(joint_positions)


def main(args=None):  # Defining the 'Main' Function
    # Confirmation that Main Functions Properly
    print('Starting Robot Control')
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()  # Establish the Client Node
    # Switch on Gripper
    status = True
    response = minimal_client.send_request(status)

    Controller = Controller_Node()  # Establish the Publisher Node

    try:
        rclpy.spin(Controller)  # Spin the Publisher Node
    except KeyboardInterrupt:
        pass

    # Switch off Gripper
    status = False
    response = minimal_client.send_request(status)

    minimal_client.destroy_node()

    Controller.destroy_node()  # Destroy node when Ctrl. C is pressed
    rclpy.shutdown()  # Shut the Node Down


if __name__ == '__main__':
    main()
