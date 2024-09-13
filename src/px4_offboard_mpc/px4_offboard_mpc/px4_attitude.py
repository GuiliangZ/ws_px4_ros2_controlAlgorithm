import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from px4_msgs.msg import VehicleLocalPosition
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import threading
import time
import numpy as np
import scvx_class as sc

class VehiclePositionSubscriber(Node):
    def __init__(self, data_lock, data_shared):
        super().__init__('vehicle_position_subscriber')
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.data_lock = data_lock
        self.data_shared = data_shared
        self.subscription = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.position_callback,
            qos)

    def position_callback(self, msg):
        if msg.xy_valid and msg.z_valid:
            with self.data_lock:
                if not self.data_shared.get('updated', False):  # Skip if already updated and not yet plotted
                    self.data_shared['x'] = msg.x
                    self.data_shared['y'] = msg.y
                    self.data_shared['z'] = -msg.z
                    self.data_shared['vx'] = msg.vx
                    self.data_shared['vy'] = msg.vy
                    self.data_shared['vz'] = msg.vz
                    self.data_shared['updated'] = True

def plot_thread(data_lock, data_shared):
    fig = plt.figure(figsize=(12, 6))
    ax1 = fig.add_subplot(121, projection='3d')
    ax2 = fig.add_subplot(122)
    ax1.set_xlabel('X Position (m)')
    ax1.set_ylabel('Y Position (m)')
    ax1.set_zlabel('Z Position (m)')
    ax1.plot_surface(sc.xe, sc.ye, sc.ze, color='r', alpha=0.6)


    ax2.set_xlabel('Time')
    ax2.set_ylabel('Velocity (m/s)')
    ax2.set_title('Velocity Profile')
    ax2.grid(True)
    create_ground_plane(ax1, x_range=(-4, 4), y_range=(-4, 4))
    
    scatter = ax1.scatter([], [], [], color='b', s=100)  # Initialize empty scatter plot
    line, = ax1.plot([], [], [], 'k-', label='Trajectory')  # Initialize empty line plot for trajectory
    positions, velocities = [], []

    plt.ion()
    plt.show()

    time_start = time.time()

    while plt.fignum_exists(fig.number):
        with data_lock:
            if data_shared.get('updated', False):
                x, y, z = data_shared['x'], data_shared['y'], data_shared['z']
                vx, vy, vz = data_shared['vx'], data_shared['vy'], data_shared['vz']
                positions.append((x, y, z))
                velocities.append((time.time() - time_start, vx, vy, vz))
                
                # Check if the point is within the sphere
                x_c, y_c, z_c = sc.centre  # Sphere center
                radius = sc.a  # Sphere radius
                distance_squared = (x - x_c) ** 2 + (y - y_c) ** 2 + (z - z_c) ** 2

                if distance_squared <= 0.8*(radius ** 2):
                    scatter_color = 'r'  # Inside sphere, use red color
                else:
                    scatter_color = 'b'  # Outside sphere, use blue color

                ax1.clear()
                create_ground_plane(ax1, x_range=(-4, 4), y_range=(-4, 4))
                scatter = ax1.scatter(x, y, z, color=scatter_color, s=100)
                xs, ys, zs = zip(*positions)
                line.set_data(np.array(xs), np.array(ys))
                line.set_3d_properties(np.array(zs))
                ax1.add_line(line)  # Update the line representing the trajectory
                ax1.set_xlim(-4, 4)
                ax1.set_ylim(-4, 4)
                ax1.set_zlim(0, 8)
                ax1.text2D(0.05, 0.95, f"Pos: ({x:.2f}, {y:.2f}, {z:.2f})\nVel: ({vx:.2f}, {vy:.2f}, {vz:.2f})", transform=ax1.transAxes)
                ax1.plot_surface(sc.xe, sc.ye, sc.ze, color='r', alpha=0.6)

                ax2.clear()
                times, vx_vals, vy_vals, vz_vals = zip(*velocities)
                ax2.plot(times, vx_vals, label='Vx (m/s)')
                ax2.plot(times, vy_vals, label='Vy (m/s)')
                ax2.plot(times, vz_vals, label='Vz (m/s)')
                ax2.legend(loc='upper right')
                ax2.grid(True)
                
                data_shared['updated'] = False

        plt.draw()
        plt.pause(0.01)
        #time.sleep(0.1)

def create_ground_plane(ax, x_range, y_range):
    """ Creates a ground plane at z=0 for the given axes within the specified range. """
    x = np.linspace(*x_range, 50)
    y = np.linspace(*y_range, 50)
    X, Y = np.meshgrid(x, y)
    Z = np.zeros_like(X)
    ax.plot_surface(X, Y, Z, color='gray', alpha=0.5)

def main(args=None):
    rclpy.init(args=args)
    data_lock = threading.Lock()
    data_shared = {'updated': False}

    node = VehiclePositionSubscriber(data_lock, data_shared)
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    plot_thread(data_lock, data_shared)

    node.destroy_node()
    rclpy.shutdown()
    thread.join()

if __name__ == '__main__':
    main()
