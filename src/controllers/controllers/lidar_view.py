import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from time import time
import math

class LidarAnalysisNode(Node):
    def __init__(self):
        super().__init__('lidar_analysis_node')
        self.subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.last_time = None
        self.freq_samples = []
        self.msg_count = 0

    def scan_callback(self, msg: LaserScan):
        # Calcular resolución angular (en grados)
        angle_increment_deg = math.degrees(msg.angle_increment)

        # Calcular frecuencia estimada
        current_time = time()
        if self.last_time is not None:
            dt = current_time - self.last_time
            self.freq_samples.append(1.0 / dt)
        self.last_time = current_time

        self.msg_count += 1
        if self.msg_count < 10:
            return  # Esperar a tener algunas muestras para estimar frecuencia

        avg_freq = sum(self.freq_samples) / len(self.freq_samples)

        # Separar rangos en frontales (0° a 180°) y traseros (180° a 360°)
        num_measurements = len(msg.ranges)
        mid_index = num_measurements // 2
        front_ranges = msg.ranges[:mid_index + 1]
        rear_ranges = msg.ranges[mid_index + 1:]

        # Mostrar resultados
        self.get_logger().info(f"Resolución angular del LiDAR: {angle_increment_deg:.3f} grados")
        self.get_logger().info(f"Frecuencia del LiDAR: {avg_freq:.2f} Hz")
        self.get_logger().info(f"Cantidad de mediciones frontales (0° a 180°): {len(front_ranges)}")
        self.get_logger().info(f"Cantidad de mediciones traseras (180° a 360°): {len(rear_ranges)}")
        self.get_logger().info(f"Primeros valores frontales: {front_ranges[:5]}")
        self.get_logger().info(f"Primeros valores traseros: {rear_ranges[:5]}")

        # Cerrar el nodo automáticamente (opcional)
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = LidarAnalysisNode()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()
