import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
import time

class FollowGap(Node):
    def __init__(self):
        super().__init__('follow_gap_node')

        # Suscripciones y publicaciones
        self.odom_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        # Variables internas
        self.last_log_time = 0.0
        self.log_interval = 1.5  # segundos
        self.robot_position = None

        # Parámetros para detección pared/obstáculo
        self.detection_distance = 1.5       # distancia máxima para considerar un objeto cercano
        self.min_arc_length_wall = 0.7      # umbral de longitud para considerar una "pared"
        self.min_group_points = 3           # número mínimo de puntos para filtrar ruido

    def odom_callback(self, msg):
        current_time = time.time()
        if current_time - self.last_log_time >= self.log_interval:
            position = msg.pose.pose.position
            self.robot_position = (position.x, position.y)
            self.get_logger().info(f'Posición del robot: x={position.x:.2f}, y={position.y:.2f}')
            self.last_log_time = current_time

    def lidar_callback(self, scan_msg):
        ranges = np.array(scan_msg.ranges)
        angle_min = scan_msg.angle_min
        angle_increment = scan_msg.angle_increment

        # Filtrar valores inválidos
        ranges = np.clip(ranges, 0, scan_msg.range_max)
        ranges[np.isnan(ranges)] = 0.0
        ranges[ranges < 0.3] = 0.0  # Ignorar obstáculos muy cercanos

        current_time = time.time()

        # Detección de objetos/paredes
        close_mask = (ranges > 0.0) & (ranges <= self.detection_distance)
        objects = []
        start = None
        for i, close in enumerate(close_mask):
            if close and start is None:
                start = i
            elif not close and start is not None:
                if i - start >= self.min_group_points:
                    objects.append((start, i - 1))
                start = None
        if start is not None and (len(ranges) - start) >= self.min_group_points:
            objects.append((start, len(ranges) - 1))

        # Clasificar objetos
        pared_detectada = False
        objeto_detectado = False
        for (start_idx, end_idx) in objects:
            angle_width = (end_idx - start_idx + 1) * angle_increment
            arc_length = angle_width * self.detection_distance  # longitud aproximada

            if arc_length >= self.min_arc_length_wall:
                pared_detectada = True
            else:
                objeto_detectado = True

        # Mostrar mensaje de detección cada 1.5 segundos
        if current_time - self.last_log_time >= self.log_interval:
            if pared_detectada:
                self.get_logger().info('Pared detectada')
            elif objeto_detectado:
                self.get_logger().info('Objeto detectado')
            else:
                self.get_logger().info('No se detectaron objetos cercanos')

        # Búsqueda de gap más grande
        gap_threshold = 2.5
        gap_mask = ranges > gap_threshold

        gaps = []
        start = None
        for i, valid in enumerate(gap_mask):
            if valid and start is None:
                start = i
            elif not valid and start is not None:
                gaps.append((start, i - 1))
                start = None
        if start is not None:
            gaps.append((start, len(ranges) - 1))

        if not gaps:
            self.get_logger().info("No se encontraron gaps seguros.")
            return

        best_gap = max(gaps, key=lambda g: g[1] - g[0])
        gap_center = (best_gap[0] + best_gap[1]) // 2
        angle = angle_min + gap_center * angle_increment

        # Velocidad basada en cercanía
        valid_ranges = ranges[ranges > 0.0]
        min_distance = np.min(valid_ranges) if len(valid_ranges) > 0 else scan_msg.range_max

        if min_distance > 1.5:
            speed = 3.0
        elif 1.5 > min_distance > 0.5:
            speed = 0.8
        else:
            speed = 0.0

        # Publicar comando
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = float(angle)
        drive_msg.drive.speed = float(speed)
        self.drive_pub.publish(drive_msg)

        # Mensaje de velocidad cada 1.5 s
        if current_time - self.last_log_time >= self.log_interval:
            self.get_logger().info(f'Velocidad actual: {speed:.2f} m/s')
            self.last_log_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = FollowGap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
