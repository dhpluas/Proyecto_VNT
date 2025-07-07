import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from math import sqrt, atan2, pi

class SquareDriver(Node):
    def __init__(self):
        super().__init__('square_driver')

        self.subscription = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        self.state = 'forward1'
        self.initial_position = None
        self.initial_yaw = None

        # Distancias a recorrer para cada etapa
        self.distances = {
            'forward1': 8.7,
            'forward2': 7.3,
            'forward3': 22.0,
            'forward4': 7.3,
            'forward5': 15.0,   # Nuevo avance final
        }

        # Tolerancias para distancia y ángulo
        self.distance_tolerance = 0.05  # metros
        self.angle_tolerance = 0.05  # radianes (~3 grados)

        # Para giros: almacenamos el ángulo objetivo absoluto
        self.target_yaw = None

        self.get_logger().info("Iniciando movimiento: avanzar 8.7 m en línea recta.")

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = self.quaternion_to_yaw(q)

        drive_msg = AckermannDriveStamped()

        # Inicializar posición o yaw para cada estado
        if self.state.startswith('forward'):
            if self.initial_position is None:
                self.initial_position = (x, y)
                self.get_logger().info(f"[{self.state}] Posición inicial: ({x:.2f}, {y:.2f})")
                return
        elif self.state.startswith('turn_left'):
            if self.initial_yaw is None:
                self.initial_yaw = yaw
                self.target_yaw = self.normalize_angle(self.initial_yaw + pi/2)
                self.get_logger().info(f"[{self.state}] Yaw inicial: {self.initial_yaw:.2f} rad, objetivo: {self.target_yaw:.2f} rad")
                return

        # Estados de avance
        if self.state.startswith('forward'):
            dist_traveled = self.distance_traveled(x, y)
            target_dist = self.distances[self.state]

            if dist_traveled < target_dist - self.distance_tolerance:
                drive_msg.drive.speed = 1.5
                drive_msg.drive.steering_angle = 0.0
                self.get_logger().info(f"[{self.state}] Recorrido: {dist_traveled:.2f} m / {target_dist} m")
            else:
                self.get_logger().info(f"[{self.state}] Meta alcanzada.")
                self.next_state()

        # Estados de giro
        elif self.state.startswith('turn_left'):
            yaw_error = self.angle_diff(yaw, self.target_yaw)

            if abs(yaw_error) > self.angle_tolerance:
                drive_msg.drive.speed = 0.3
                drive_msg.drive.steering_angle = 0.4
                self.get_logger().info(f"[{self.state}] Girando... error: {yaw_error:.2f} rad")
            else:
                self.get_logger().info(f"[{self.state}] Giro completo.")
                self.next_state()

        elif self.state == 'done':
            drive_msg.drive.speed = 0.0
            drive_msg.drive.steering_angle = 0.0
            self.get_logger().info("[done] Ruta completada. Vehículo detenido.")

        self.publisher.publish(drive_msg)

    def next_state(self):
        # Avanzar a siguiente estado según el orden
        if self.state == 'forward1':
            self.state = 'turn_left1'
        elif self.state == 'turn_left1':
            self.state = 'forward2'
        elif self.state == 'forward2':
            self.state = 'turn_left2'
        elif self.state == 'turn_left2':
            self.state = 'forward3'
        elif self.state == 'forward3':
            self.state = 'turn_left3'
        elif self.state == 'turn_left3':
            self.state = 'forward4'
        elif self.state == 'forward4':
            self.state = 'turn_left4'
        elif self.state == 'turn_left4':
            self.state = 'forward5'
        elif self.state == 'forward5':
            self.state = 'done'
        else:
            self.state = 'done'

        self.initial_position = None
        self.initial_yaw = None
        self.target_yaw = None

        self.get_logger().info(f"[TRANSICIÓN] Nuevo estado: {self.state}")

    def distance_traveled(self, x, y):
        dx = x - self.initial_position[0]
        dy = y - self.initial_position[1]
        return sqrt(dx**2 + dy**2)

    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return atan2(siny_cosp, cosy_cosp)

    def angle_diff(self, current, target):
        diff = current - target
        while diff > pi:
            diff -= 2 * pi
        while diff < -pi:
            diff += 2 * pi
        return diff

    def normalize_angle(self, angle):
        while angle > pi:
            angle -= 2 * pi
        while angle < -pi:
            angle += 2 * pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = SquareDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
