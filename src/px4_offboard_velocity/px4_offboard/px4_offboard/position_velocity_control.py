#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleAttitude
from geometry_msgs.msg import Vector3, Twist
import numpy as np
from dataclasses import dataclass
from typing import Dict, Optional, List
import threading
from rclpy.executors import MultiThreadedExecutor


# Define a class for individual points
@dataclass
class Point:
    x: float
    y: float
    z: float

# Define a class for setpoints containing multiple points
@dataclass
class Setpoint:
    points: List[Point]

    def __init__(self, a: Point, b: Point, c: Point, d: Point, e: Point, f: Point, g: Point):
        self.points = [a, b, c, d, e, f, g]

    def pop_point(self, index: int) -> Optional[Point]:
        if 0 <= index < len(self.points):
            return self.points.pop(index)
        return None

    def get_point(self, index: int) -> Optional[Point]:
        if 0 <= index < len(self.points):
            return self.points[index]
        return None

class PositionVelocityControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('position_velocity_control')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.attitude_sub = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude', self.attitude_callback, qos_profile)
        
        # Start a separate thread for the main processing loop
        self.thread = threading.Thread(target=self.timer_callback)
        self.thread.start()
        
        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.takeoff_counter = 0
        self.position_reached_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -5.0
        self.yaw = 0.0  #yaw value we send as command
        self.trueYaw = 0.0  #current yaw value of drone
        # Define PD controller parameters
        self.Kp = 0.2
        self.Ki = 0.03
        self.Kd = 0.03

        self.dt = 0.02
        # Error terms
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.prev_error_z = 0.0
        self.integral_error_x = 0.0
        self.integral_error_y = 0.0
        self.integral_error_z = 0.0

        # Define velocity cap limit
        self.velocity_limit = 1

        # Define position precision
        self.position_precision = 0.5

        #staged position waypoint reached
        self.position_reached = False
        self.position_control_mode = True
        self.initial_position_point = True

        #set the current flight modes
        self.control_mode = 'takeoff' #Modes: 'position', 'MPC', 'PD-control' 'landing'

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.02, self.timer_callback)

        #The Y, Z is flipped from the body frame to the world(vicon) frame
        # eg. point(3,-3,-5) is (3,3,5) in the world frame
        # The setpoints given here is in the body frame 
        self.setpoints = Setpoint(
            a=Point(0.0, 0.0, -5.0),
            b=Point(3.0, -3.0, -5.0),
            c=Point(-3.0, -3.0, -5.0),
            d=Point(-3.0, 3.0, -5.0),
            e=Point(3.0, 3.0, -5.0),
            f=Point(3.0, -3.0, -5.0),
            g=Point(0.0, 0.0, -5.0),
        )

        self.target_position = Point(0.0,0.0,-5.0)


    """Drone action functions"""
    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")


    """Subscriber callback functions"""
    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    #receives current trajectory values from drone and grabs the yaw value of the orientation
    def attitude_callback(self, msg):
        """Callback function for vehicle_attitude topic subscriber."""
        orientation_q = msg.q
        #trueYaw is the drones current yaw value
        self.trueYaw = -(np.arctan2(2.0*(orientation_q[3]*orientation_q[0] + orientation_q[1]*orientation_q[2]), 
                                  1.0 - 2.0*(orientation_q[0]*orientation_q[0] + orientation_q[1]*orientation_q[1])))
        
    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position


    """Publisher functions"""
    #This tells the PX4, we are publishing velocity setpoints when using offboard mode
    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    # def publish_takeoff(self):
    #     """Publish the trajectory setpoint."""
    #     msg = TrajectorySetpoint()
    #     msg.position = [0.0, 0.0, self.takeoff_height]
    #     msg.yaw = 1.57079  # (90 degree)
    #     msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
    #     self.trajectory_setpoint_publisher.publish(msg)
    #     self.get_logger().info(f"Taking Off!")

    # This is not used for velocity control
    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    # This is not used for velocity control
    def publish_position_setpoint(self, x: float, y: float, z: float):
        """Publish the position trajectory setpoint."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 1.57079  # (90 degree)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")
        self.get_logger().info(f"Local Position {[self.vehicle_local_position.x, self.vehicle_local_position.y, self.vehicle_local_position.z]}")

    def velocity_position_controller(self, target_position):
        "Calculate the velocity at each point given target position"
        "Implement PID controller for X direction and PI controller for the Y, Z direction."
        #use vehicle local position from FC
        velocity_yaw = Twist()
        #You can only control the angular rate at z direction - yaw. This is partially attitude control. 
        velocity_yaw.angular.x = 0.0
        velocity_yaw.angular.y = 0.0
        velocity_yaw.angular.z = 0.0

        #change the vehicle body coordinates to the vicon reference coordinates
        # self.get_logger().info(f"Moving to setpoint: x={target_position.x}, y={target_position.y}, z={target_position.z}")
        # self.get_logger().info(f"Local Position {[self.vehicle_local_position.x, self.vehicle_local_position.y, self.vehicle_local_position.z]}")
        #implementing PID controller
        error_x = target_position.x - self.vehicle_local_position.x
        error_y = target_position.y - self.vehicle_local_position.y
        error_z = target_position.z - self.vehicle_local_position.z

        # Integral term
        self.integral_error_x += error_x * self.dt
        self.integral_error_y += error_y * self.dt
        self.integral_error_z += error_z * self.dt

        # derivative error
        derivative_error_x = (error_x - self.prev_error_x) / self.dt
        derivative_error_y = (error_x + self.prev_error_y) / self.dt
        derivative_error_z = (error_x + self.prev_error_z) / self.dt
        
        #the desired velocity is under world frame
        desired_vel_x = self.Kp * error_x + self.Ki * self.integral_error_x + self.Kd * derivative_error_x
        #desired_vel_y = self.Kp * error_y + self.Ki * self.integral_error_y + self.Kd * derivative_error_y
        desired_vel_y = self.Kp * error_y + self.Ki * self.integral_error_y
        desired_vel_z = self.Kp * error_z + self.Ki * self.integral_error_z 
        # desired_vel_z = self.Kp * error_z + self.Ki * self.integral_error_z 
        # desired_vel_y = self.Kp * error_y + self.Ki * self.integral_error_y + self.Kd * derivative_error_y
        # desired_vel_y = self.Kp * error_y
        # desired_vel_z = self.Kp * error_z
        # print("des_X", desired_vel_x, " des_Y", desired_vel_y, " des_Z", desired_vel_z, " /")

        # Update previous error
        self.prev_error_x = error_x
        self.prev_error_y = error_y
        self.prev_error_z = error_z

        self.get_logger().info(f"Desired velocity setpoints {[desired_vel_x, desired_vel_y, desired_vel_z]}")
        if abs(desired_vel_x) > self.velocity_limit:
            velocity_yaw.linear.x = np.sign(desired_vel_x) * self.velocity_limit
        elif abs(desired_vel_x) <= self.velocity_limit:
            velocity_yaw.linear.x = desired_vel_x
        if abs(desired_vel_y) > self.velocity_limit:
            velocity_yaw.linear.y = np.sign(desired_vel_y) * self.velocity_limit
        elif abs(desired_vel_y) <= self.velocity_limit:
            velocity_yaw.linear.y = desired_vel_y
        if abs(desired_vel_z) > self.velocity_limit:
            velocity_yaw.linear.z = np.sign(desired_vel_z) * self.velocity_limit
        elif abs(desired_vel_z) <= self.velocity_limit:
            velocity_yaw.linear.z = desired_vel_z
        # print("pos_X", local_position_msg.x, " pos_Y", local_position_msg.y, " pos_Z", local_position_msg.z, " /")
        print("velcmd_X:",velocity_yaw.linear.x, "   velcmd_Y:",velocity_yaw.linear.y, "   velcmd_Z:",velocity_yaw.linear.z, "   velcmd_Yaw:",velocity_yaw.angular.z, " /")
        print("Next /")
        return velocity_yaw
    
    def publish_velocity_setpoint(self, target_position):
        "Take current target position and publish the velocity setpoints"
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

        velocity_setpoints = Twist()
        velocity_setpoints = self.velocity_position_controller(target_position)
        # Compute velocity in the world frame
        cos_yaw = np.cos(self.trueYaw)
        sin_yaw = np.sin(self.trueYaw)
        velocity_world_x = (velocity_setpoints.linear.x * cos_yaw - velocity_setpoints.linear.y * sin_yaw)
        velocity_world_y = (velocity_setpoints.linear.x * sin_yaw + velocity_setpoints.linear.y * cos_yaw)

        # Create and publish TrajectorySetpoint message with NaN values for position and acceleration
        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        # trajectory_msg.velocity[0] = velocity_world_x
        # trajectory_msg.velocity[1] = velocity_world_y
        trajectory_msg.velocity[0] = velocity_setpoints.linear.x
        trajectory_msg.velocity[1] = velocity_setpoints.linear.y
        trajectory_msg.velocity[2] = velocity_setpoints.linear.z
        trajectory_msg.position[0] = float('nan')
        trajectory_msg.position[1] = float('nan')
        trajectory_msg.position[2] = float('nan')
        trajectory_msg.acceleration[0] = float('nan')
        trajectory_msg.acceleration[1] = float('nan')
        trajectory_msg.acceleration[2] = float('nan')
        trajectory_msg.yaw = float('nan')
        trajectory_msg.yawspeed = self.yaw #0 in this case
        self.trajectory_setpoint_publisher.publish(trajectory_msg)
        self.get_logger().info(f"Publishing velocity setpoints {[velocity_setpoints.linear.x, velocity_setpoints.linear.y, velocity_setpoints.linear.z]}")

    def direct_publish_velocity_setpoint(self, velocity_x, velocity_y, velocity_z):
            "Directly publish the velocity setpoints"
            msg = OffboardControlMode()
            msg.position = False
            msg.velocity = True
            msg.acceleration = False
            msg.attitude = False
            msg.body_rate = False
            msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.offboard_control_mode_publisher.publish(msg)

            # Create and publish TrajectorySetpoint message with NaN values for position and acceleration
            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000)
            # trajectory_msg.velocity[0] = velocity_world_x
            # trajectory_msg.velocity[1] = velocity_world_y
            trajectory_msg.velocity[0] = velocity_x
            trajectory_msg.velocity[1] = velocity_y
            trajectory_msg.velocity[2] = velocity_z
            trajectory_msg.position[0] = float('nan')
            trajectory_msg.position[1] = float('nan')
            trajectory_msg.position[2] = float('nan')
            trajectory_msg.acceleration[0] = float('nan')
            trajectory_msg.acceleration[1] = float('nan')
            trajectory_msg.acceleration[2] = float('nan')
            trajectory_msg.yaw = float('nan')
            trajectory_msg.yawspeed = self.yaw #0 in this case
            self.trajectory_setpoint_publisher.publish(trajectory_msg)
            self.get_logger().info(f"Publishing velocity setpoints {[velocity_x, velocity_y, velocity_z]}")

#check if the target position has reached, if not, keep publishing velocity setpoints to track target position
    def position_setpoint_track(self, target_position, local_position): 
        if np.linalg.norm(np.array([self.target_position.x, self.target_position.y, self.target_position.z]) - \
                              np.array([local_position.x, local_position.y, local_position.z])) > 0.5:
                self.get_logger().info(f"Moving to setpoint: x={target_position.x}, y={target_position.y}, z={target_position.z}")
                self.get_logger().info(f"Local Position {[local_position.x, local_position.y, local_position.z]}")
                self.publish_velocity_setpoint(target_position)
                self.get_logger().info(f"Next: Local Position {[local_position.x, local_position.y, local_position.z]}")
        else:
            if self.position_reached_counter < 100:
                self.position_reached_counter += 1
                self.get_logger().info(f"Achieved!! setpoint: x={target_position.x}, y={target_position.y}, z={target_position.z}")
                self.direct_publish_velocity_setpoint(0.0,0.0,0.0)
                print(self.position_reached_counter)
                print(self.position_reached)
            if self.position_reached_counter == 100:
                self.position_reached = True
                self.position_reached_counter = 0
                # Initialize PID Error terms
                self.prev_error_x = 0.0
                self.prev_error_y = 0.0
                self.prev_error_z = 0.0
                self.integral_error_x = 0.0
                self.integral_error_y = 0.0
                self.integral_error_z = 0.0

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()

        if self.control_mode == 'takeoff' and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            # self.publish_takeoff()
            self.publish_position_setpoint(0.0,0.0,self.takeoff_height)
            self.takeoff_counter += 1
            if self.takeoff_counter > 10: #and self.vehicle_local_position.z - self.takeoff_height < 0.1:
                self.control_mode = 'position'
        elif self.control_mode == 'position' and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            if self.initial_position_point:
                self.target_position = self.setpoints.pop_point(0)
                self.initial_position_point = False
            self.publish_position_setpoint(self.target_position.x, self.target_position.y, self.target_position.z)
            # if np.linalg.norm(self.target_position - self.vehicle_local_position) < 0.1:
            # vehicle_position = np.array([self.vehicle_local_position.x, self.vehicle_local_position.y, self.vehicle_local_position.z])
            # target_position_array = np.array([self.target_position.x, self.target_position.y, self.target_position.z])
            if np.linalg.norm(np.array([self.target_position.x, self.target_position.y, self.target_position.z]) - \
                              np.array([self.vehicle_local_position.x, self.vehicle_local_position.y, self.vehicle_local_position.z])) < 0.5:
                print("!!!First desired position setpoint has reached")
                self.position_reached = True
                self.control_mode = 'velocity'
        elif self.control_mode == 'velocity' and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            if self.position_reached:
                if not self.setpoints.points: #check if the setpoints is already empty
                    self.control_mode = 'landing'
                elif self.setpoints.points:  #keep poping out the setpoint when the setpoints list is not empty
                    self.target_position = self.setpoints.pop_point(0)
                    self.position_reached = False
            self.position_setpoint_track(self.target_position, self.vehicle_local_position)
        elif self.control_mode == 'landing' and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.land()
            exit(0)

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1


def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)

    position_velocity = PositionVelocityControl()

    rclpy.spin(position_velocity)

    position_velocity.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)