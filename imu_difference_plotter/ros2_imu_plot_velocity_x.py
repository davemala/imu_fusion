import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy  # Spostato qui
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
import numpy as np
from collections import deque
import threading

class IMUDifferencePlotter(Node):
    def __init__(self):
        super().__init__('imu_difference_plotter')

        self.media = 0

        # Imposta il QoS come BEST_EFFORT per garantire compatibilità
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # Aggiorna i topic con quelli corretti
        self.subscription_imu1 = self.create_subscription(
            Imu,
            '/imu/data',  # Nome corretto del primo topic
            self.imu1_callback,
            qos_profile)

        self.subscription_imu2 = self.create_subscription(
            Imu,
            '/ouster/imu',  # Nome corretto del secondo topic
            self.imu2_callback,
            qos_profile)

        self.imu1_data = None
        self.imu2_data = None
        self.time_window = 100  # Numero di campioni da visualizzare
        
        self.imu1_values = deque(maxlen=self.time_window)  # Dati IMU1
        self.imu2_values = deque(maxlen=self.time_window)  # Dati IMU2
        self.media_values = deque(maxlen=self.time_window)  # Media

        self.diff_data = deque(maxlen=self.time_window)
        self.time_axis = deque(maxlen=self.time_window)
        self.counter = 0
        
        self.lock = threading.Lock()
        self.plot_thread = threading.Thread(target=self.plot_loop)
        self.plot_thread.daemon = True
        self.plot_thread.start()

    def imu1_callback(self, msg):
        self.imu1_data = msg.angular_velocity.x  # Usa un asse specifico
        self.compute_difference()

    def imu2_callback(self, msg):
        self.imu2_data = msg.angular_velocity.x  # Usa lo stesso asse
        #self.compute_difference()

    def compute_difference(self):
        with self.lock:
            if self.imu1_data is not None and self.imu2_data is not None:
                self.imu1_values.append(self.imu1_data)
                self.imu2_values.append(self.imu2_data)
                
                diff = self.imu1_data - self.imu2_data
                self.media = (self.media * 99 + diff) / 100
                self.diff_data.append(diff)
                self.media_values.append(self.media)
                self.time_axis.append(self.counter)
                self.counter += 1


    def plot_loop(self):
        plt.ion()
        fig, ax = plt.subplots()
        
        # Tre linee: IMU1, IMU2 e la differenza
        line_imu1, = ax.plot([], [], 'b-', label="IMU 1")  # Blu
        line_imu2, = ax.plot([], [], 'g-', label="IMU 2")  # Verde
        line_diff, = ax.plot([], [], 'r-', label="Difference")  # Rosso
        line_med, = ax.plot([], [], 'y-', label="Media")

        ax.set_xlabel('Samples')
        ax.set_ylabel('IMU Values')
        ax.set_title('Real-time IMU Data and Difference')
        ax.legend()

        while rclpy.ok():
            with self.lock:
                line_imu1.set_xdata(np.array(self.time_axis))
                line_imu1.set_ydata(np.array(self.imu1_values))

                line_imu2.set_xdata(np.array(self.time_axis))
                line_imu2.set_ydata(np.array(self.imu2_values))

                line_diff.set_xdata(np.array(self.time_axis))
                line_diff.set_ydata(np.array(self.diff_data))

                line_med.set_xdata(np.array(self.time_axis))
                line_med.set_ydata(np.array(self.media_values))

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
