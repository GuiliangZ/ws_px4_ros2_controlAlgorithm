import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
import numpy as np
import cvxpy as cp
import time


class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_land')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)  # tells what aspect are you controlling, uint64 timestamp [position, velocity, acceleration, attitude, body_rate, thrust_and_torque, direct_actuator
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.vehicle_command_subscriber = self.create_subscription(
            TrajectorySetpoint, '/local_vehicle_command_control', self.local_vehicle_command_control_callback, qos_profile)  # takes command from actual controller

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.local_vehicle_command_control = TrajectorySetpoint()
        self.takeoff_height = 10
        self.takeoff_X = 0
        self.takeoff_Y = 0
        self.takeoff_counter = 0

        self.pos=True
        self.vel=False

        self.xlim = 10*np.array([-2,2])
        self.ylim = 10*np.array([-2,2])
        self.zlim = 10*np.array([-2,2])
        self.vlim = 0.2
        self.alim = 8

        self.current_time = time.time()
        self.callback_time = time.time()
        self.flag_in = 0
        # Create a timer to publish control commands
        self.timer = self.create_timer(0.01, self.timer_callback)

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    def local_vehicle_command_control_callback(self, local_vehicle_command_control):
        """Callback function for vehicle_local_position topic subscriber."""
        self.flag_in = 1
        self.callback_time = time.time()
        self.local_vehicle_command_control = local_vehicle_command_control

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

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

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = self.pos
        msg.velocity = self.vel
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, -z]
        msg.yaw = 1.57079  # (90 degree)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        #self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

    def publish_trajectory_setpoint(self, x: float, y: float, z: float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [float('nan')] * 3
        msg.velocity = [x, y, -z]
        msg.yaw = 1.57079  # (90 degree)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing velocity setpoints {[x, y, z]}")



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

    def within_limits(self, vector, limits):
        """ Check if vector components are within the provided limits. """
        for v, lim in zip(vector, limits):
            if isinstance(lim, np.ndarray):
                if v < lim[0] or v > lim[1]:
                    return False
            else:  # Assuming lim is a scalar applying to both negative and positive limits
                if abs(v) > lim:
                    return False
        return True

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()
        
        if (time.time()-self.callback_time)>1 and self.flag_in==1:
            self.land()
            exit(0)

        current_pos = np.array([self.vehicle_local_position.x,
                            self.vehicle_local_position.y,
                            -self.vehicle_local_position.z])

        current_vel = np.array([self.vehicle_local_position.vx,
                            self.vehicle_local_position.vy,
                            -self.vehicle_local_position.vz])

        current_acc = np.array([self.vehicle_local_position.ax,
                            self.vehicle_local_position.ay,
                            -self.vehicle_local_position.az])
        
        # Check position limits
        if not self.within_limits(current_pos, [self.xlim, self.ylim, self.zlim]):
            self.get_logger().info("Position Out of Bound! Landing!")
            self.land()
            exit(0)


        # Check velocity magnitude against the scalar limit
        '''if np.linalg.norm(current_vel) > self.vlim:
            self.get_logger().info("Velocity Out of Bound! Landing!")
            self.land()
            exit(0)

        # Check velocity magnitude against the scalar limit
        if np.linalg.norm(current_acc) > self.alim:
            self.get_logger().info("Acceleration Out of Bound! Landing!")
            self.land()
            exit(0)'''

        #print(self.local_vehicle_command_control.velocity)
        if np.all(self.local_vehicle_command_control.velocity != np.array([0,0,0])):
            if np.linalg.norm(self.local_vehicle_command_control.velocity)>self.vlim:
                self.get_logger().info("Limiting Velocity!")
                print(np.linalg.norm(self.local_vehicle_command_control.velocity))
                self.local_vehicle_command_control.velocity = self.local_vehicle_command_control.velocity*self.vlim/np.linalg.norm(self.local_vehicle_command_control.velocity)
                #self.get_logger().info(np.linalg.norm(self.local_vehicle_command_control.velocity))
        self.trajectory_setpoint_publisher.publish(self.local_vehicle_command_control)
        


def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control_safeLayer = OffboardControl()
    rclpy.spin(offboard_control_safeLayer)
    offboard_control_safeLayer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)