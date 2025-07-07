import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np

class FollowGap(Node):
    def __init__(self):
        super().__init__('follow_gap_node')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )
        self.publisher = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10
        )

        self.declare_parameter("max_speed", 10.3)
        self.declare_parameter("gap_threshold", 2.5)

    def lidar_callback(self, scan_msg):
        ranges = np.array(scan_msg.ranges)
        angle_min = scan_msg.angle_min
        angle_increment = scan_msg.angle_increment

        # Filtrar valores no válidos
        ranges = np.clip(ranges, 0, scan_msg.range_max)
        ranges[np.isnan(ranges)] = 0.0
        ranges[ranges < 0.3] = 0.0

        gap_threshold = self.get_parameter("gap_threshold").value
        gap_mask = ranges > gap_threshold

        gaps = []
        start = None
        for i, valid in enumerate(gap_mask):
            if valid and start is None:
                start = i
            elif not valid and start is not None:
                gaps.append((start, i-1))
                start = None
        if start is not None:
            gaps.append((start, len(ranges) - 1))

        if not gaps:
            self.get_logger().info("No se encontraron gaps seguros.")
            return

        best_gap = max(gaps, key=lambda g: g[1] - g[0])
        gap_center = (best_gap[0] + best_gap[1]) // 2
        angle = angle_min + gap_center * angle_increment

        # Calcular la distancia media dentro del mejor gap
        gap_distances = ranges[best_gap[0]:best_gap[1]+1]
        mean_distance = np.mean(gap_distances)

        # Calcular velocidad dinámica (ej: lineal entre min_speed y max_speed)
        max_speed = self.get_parameter("max_speed").value
        min_speed = 7.0  # velocidad minima del auto
        max_distance = 11.0  # distancia para alcanzar max_speed
        speed = min_speed + (max_speed - min_speed) * min(mean_distance / max_distance, 1.0)

        # Crear y publicar mensaje
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = float(angle)
        drive_msg.drive.speed = float(speed)
        self.publisher.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FollowGap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
