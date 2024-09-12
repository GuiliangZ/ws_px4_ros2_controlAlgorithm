#!/usr/bin/env python3
import sys

import geometry_msgs.msg
import rclpy
import std_msgs.msg
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import rclpy
from rclpy.node import Node
import numpy as np
from rclpy.clock import Clock
from px4_msgs.msg import TrajectorySetpoint, VehicleLocalPosition, VehicleStatus, VehicleCommand
from dataclasses import dataclass

# Define a class for individual points
@dataclass
class Point:
    x: float
    y: float
    z: float

# Define a class for setpoints containing multiple points
@dataclass
class Setpoint:
    a: Point
    b: Point
    c: Point
    d: Point
    e: Point
    f: Point

class PositionVelocityControl(Node):

    def __init__(self):
        super().__init__('position_velocity_control')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        #Create subscriptions
        # self.status_sub = self.create_subscription(
        #     VehicleStatus,
        #     '/fmu/out/vehicle_status',
        #     self.vehicle_status_callback,
        #     qos_profile)

        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, 
            '/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback,
            qos_profile)

        # create publisher
        self.cmd_pub = self.create_publisher(geometry_msgs.msg.Twist, '/offboard_velocity_cmd', qos_profile)
        self.arm_pub = self.create_publisher(std_msgs.msg.Bool, '/arm_message', qos_profile)
        # self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)

        #start a cmdloop with 0.02 frequency
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.arm_toggle = False

        # Define PD controller parameters
        self.Kp = 0.2
        # self.kd = 0

        # Define velocity cap limit
        self.velocity_limit = 1

        #create desired location setpoints
        #(X, Y, Z) - in meters
        # Initialize the setpoints with the given coordinates
        # self.setpoints = Setpoint(
        #     a=Point(0, 0, -0.25),
        #     b=Point(0.2, -0.2, -0.25),
        #     c=Point(-0.2, -0.2, -0.25),
        #     d=Point(-0.2, 0.2, -0.25),
        #     e=Point(0.2, 0.2, -0.25),
        #     f=Point(0.2, -0.2, -0.25)
        # )

        self.setpoints = Setpoint(
            a=Point(0, 0, -5),
            b=Point(3, -3, -5),
            c=Point(-3, -3, -5),
            d=Point(-3, 3, -5),
            e=Point(3, 3, -5),
            f=Point(3, -3, -5)
        )

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position
        print("pos_X", self.vehicle_local_position.x, " pos_Y", self.vehicle_local_position.y, " pos_Z", self.vehicle_local_position.z)

    # def vehicle_status_callback(self, vehicle_status):
    #     """Callback function for vehicle_status topic subscriber."""
    #     self.vehicle_status = vehicle_status

    # def publish_position_setpoint(self, x: float, y: float, z: float):
    #     """Publish the trajectory setpoint."""
    #     msg = TrajectorySetpoint()
    #     msg.position = [x, y, z]
    #     msg.yaw = 1.57079  # (90 degree)
    #     msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
    #     self.trajectory_setpoint_publisher.publish(msg)
    #     self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

    def cmdloop_callback(self, ):
        if self.offboard_setpoint_counter == 8:
            arm_toggle = not arm_toggle  # Flip the value of arm_toggle
            arm_msg = std_msgs.msg.Bool()
            arm_msg.data = arm_toggle
            self.arm_pub.publish(arm_msg)
            print(f"Arm toggle is now: {arm_toggle}")

        # if self.vehicle_local_position.z > self.takeoff_height and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
        #     # self.publish_position_setpoint(self.vehicle_local_position.x - self.takeoff_X, self.vehicle_local_position.y + self.takeoff_Y, self.vehicle_local_position.z + self.takeoff_height)
        #     self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)

        # elif self.vehicle_local_position.z <= self.takeoff_height:
        #     self.land()
        #     exit(0)

        # self.vehicle_local_position.x
        # self.vehicle_local_position.y
        # self.vehicle_local_position.z

        # self.setpoint.a.x

        twist = geometry_msgs.msg.Twist()
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        for point_name, point in vars(self.setpoints).items():
            # print(f"Point {point_name}: x = {point.x}, y = {point.y}, z = {point.z}")
            while abs(point.x - self.vehicle_local_position.x) > 0.5 or \
                abs(point.y - self.vehicle_local_position.y) > 0.5 or \
                abs(point.z - self.vehicle_local_position.z) > 0.5:
                desired_vel_x = self.Kp * (self.setpoints.a.x - self.vehicle_local_position.x)
                desired_vel_y = self.Kp * (self.setpoints.a.y - self.vehicle_local_position.y)
                desired_vel_z = self.Kp * (self.setpoints.a.z - self.vehicle_local_position.z)
                # print("des_X", desired_vel_x, " des_Y", desired_vel_y, " des_Z", desired_vel_z, " /")
                if abs(desired_vel_x) > self.velocity_limit:
                    twist.linear.x = np.sign(desired_vel_z) * self.velocity_limit
                elif abs(desired_vel_x) <= self.velocity_limit:
                    twist.linear.x = desired_vel_x
                if abs(desired_vel_y) > self.velocity_limit:
                    twist.linear.y = np.sign(desired_vel_z) * self.velocity_limit
                elif abs(desired_vel_y) <= self.velocity_limit:
                    twist.linear.y = desired_vel_y
                if abs(desired_vel_z) > self.velocity_limit:
                    twist.linear.z = np.sign(desired_vel_z) * self.velocity_limit
                elif abs(desired_vel_z) <= self.velocity_limit:
                    twist.linear.z = desired_vel_z

                self.cmd_pub.publish(twist)
                # print("pos_X", self.vehicle_local_position.x, " pos_Y", self.vehicle_local_position.y, " pos_Z", self.vehicle_local_position.z, " /")
                # print("velcmd_X:",twist.linear.x, "   velcmd_Y:",twist.linear.y, "   velcmd_Z:",twist.linear.z, "   velcmd_Yaw:",twist.angular.z, " /")
                # print("Next /")
        if self.offboard_setpoint_counter < 9:
            self.offboard_setpoint_counter += 1


def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)

    position_velocity = PositionVelocityControl()

    rclpy.spin(position_velocity)

    position_velocity.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()