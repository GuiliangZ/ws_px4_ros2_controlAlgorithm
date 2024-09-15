import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from vicon_receiver.msg import Position
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

def quaternion_to_rotation_matrix(q):
    """Convert a quaternion into a rotation matrix."""
    q0, q1, q2, q3 = q
    return np.array([
        [1 - 2*(q2**2 + q3**2), 2*(q1*q2 - q3*q0), 2*(q1*q3 + q2*q0)],
        [2*(q1*q2 + q3*q0), 1 - 2*(q1**2 + q3**2), 2*(q2*q3 - q1*q0)],
        [2*(q1*q3 - q2*q0), 2*(q2*q3 + q1*q0), 1 - 2*(q1**2 + q2**2)]
    ])

class ViconSubscriber(Node):
    def __init__(self):
        super().__init__('vicon_subscriber')
        self.subscription = self.create_subscription(
            Position,
            '/vicon/Droplet2/Droplet2',
            self.position_callback,
            2)
        self.subscription  # prevent unused variable warning
        self.current_position = None
        self.current_orientation = None

        # Setting up the 3D plot
        plt.ion()  # Turn on interactive mode for live updates
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlim(-2000, 2000)
        self.ax.set_ylim(-2000, 2000)
        self.ax.set_zlim(-2000, 2000)
        self.ax.set_xlabel('X Position')
        self.ax.set_ylabel('Y Position')
        self.ax.set_zlabel('Z Position')

    def position_callback(self, msg):
        self.current_position = (msg.x_trans, msg.y_trans, msg.z_trans)
        self.current_orientation = (msg.x_rot, msg.y_rot, msg.z_rot, msg.w)
        self.update_plot()

    def update_plot(self):
        if plt.fignum_exists(self.fig.number):
            self.ax.clear()
            # Plot the current position as a point
            self.ax.scatter(*self.current_position, color='b')

            if self.current_orientation:
                # Convert quaternion to rotation matrix and plot orientation vectors
                R = quaternion_to_rotation_matrix(self.current_orientation)
                length = 1000.0  # Length of the orientation vectors
                for i, color in zip(range(3), ['r', 'g', 'b']):
                    # Extract the column of the rotation matrix as the direction vector
                    direction = R[:, i]
                    self.ax.quiver(*self.current_position, *direction, length=length, color=color)

            # Update limits if needed
            self.ax.set_xlim(-2000, 2000)
            self.ax.set_ylim(-2000, 2000)
            self.ax.set_zlim(-2000, 2000)
            self.ax.set_xlabel('X Position')
            self.ax.set_ylabel('Y Position')
            self.ax.set_zlabel('Z Position')
            plt.draw()
            plt.pause(0.01)  # Lower the pause to improve responsiveness

def main(args=None):
    rclpy.init(args=args)
    vicon_subscriber = ViconSubscriber()
    executor = SingleThreadedExecutor()

    executor.add_node(vicon_subscriber)

    try:
        while rclpy.ok() and plt.fignum_exists(vicon_subscriber.fig.number):
            executor.spin_once(timeout_sec=0.1)
            plt.pause(0.01)  # Integrate matplotlib processing into the ROS2 loop
    except KeyboardInterrupt:
        pass
    finally:
        plt.close(vicon_subscriber.fig)  # Close the figure
        vicon_subscriber.destroy_node()  # Destroy the node
        rclpy.shutdown()  # Shutdown ROS

if __name__ == '__main__':
    main()
