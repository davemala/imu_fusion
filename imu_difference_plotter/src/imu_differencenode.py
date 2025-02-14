import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
import numpy as np
from collections import deque
import threading

class IMUDifferencePlotter(Node):
    def __init__(self):
        super().__init__('imu_difference_plotter')
        
        self.subscription_imu1 = self.create_subscription(
            Imu,
            '/imu1/data',
            self.imu1_callback,
            10)
        
        self.subscription_imu2 = self.create_subscription(
            Imu,
            '/imu2/data',
            self.imu2_callback,
            10)
        
        self.imu1_data = None
        self.imu2_data = None
        self.time_window = 100  # Numero di campioni da visualizzare
        self.diff_data = deque(maxlen=self.time_window)
        self.time_axis = deque(maxlen=self.time_window)
        self.counter = 0
        
        self.lock = threading.Lock()
        self.plot_thread = threading.Thread(target=self.plot_loop)
        self.plot_thread.daemon = True
        self.plot_thread.start()

    def imu1_callback(self, msg):
        self.imu1_data = msg.angular_velocity.z  # Usa un asse specifico
        self.compute_difference()

    def imu2_callback(self, msg):
        self.imu2_data = msg.angular_velocity.z  # Usa lo stesso asse
        self.compute_difference()

    def compute_difference(self):
        with self.lock:
            if self.imu1_data is not None and self.imu2_data is not None:
                diff = self.imu1_data - self.imu2_data
                self.diff_data.append(diff)
                self.time_axis.append(self.counter)
                self.counter += 1

    def plot_loop(self):
        plt.ion()
        fig, ax = plt.subplots()
        line, = ax.plot([], [], 'r-')
        ax.set_xlabel('Samples')
        ax.set_ylabel('IMU Difference')
        ax.set_title('Real-time IMU Difference Plot')
        
        while rclpy.ok():
            with self.lock:
                line.set_xdata(np.array(self.time_axis))
                line.set_ydata(np.array(self.diff_data))
                ax.relim()
                ax.autoscale_view()
            
            plt.draw()
            plt.pause(0.1)
        
        plt.ioff()
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = IMUDifferencePlotter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
