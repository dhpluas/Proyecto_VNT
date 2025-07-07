import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import time

class LapTimer(Node):
    def __init__(self):
        super().__init__('lap_timer')

        self.subscription = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10
        )

        # Zona de línea de meta (ajustada al inicio del auto)
        self.line_x_min = -0.25
        self.line_x_max = 0.25
        self.line_y_min = -0.25
        self.line_y_max = 0.25

        # Variables del contador
        self.lap_count = 0
        self.passed_line = False
        self.lap_start_time = time.time()
        self.lap_times = []

        self.get_logger().info("🕒 Cronómetro de vueltas iniciado.")

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        in_zone = (self.line_x_min <= x <= self.line_x_max and
                   self.line_y_min <= y <= self.line_y_max)

        if in_zone and not self.passed_line:
            self.lap_count += 1
            current_time = time.time()
            lap_time = current_time - self.lap_start_time
            self.lap_start_time = current_time
            self.lap_times.append(lap_time)

            self.get_logger().info(
                f"🏁 Vuelta {self.lap_count} | Tiempo: {lap_time:.2f} segundos")

            self.passed_line = True

        elif not in_zone:
            self.passed_line = False

def main(args=None):
    rclpy.init(args=args)
    node = LapTimer()
    rclpy.spin(node)
    node.destroy_node()

    # Mostrar resumen al final
    print("\n🏁 RESUMEN DE TIEMPOS POR VUELTA:")
    for i, t in enumerate(node.lap_times, 1):
        print(f"Vuelta {i}: {t:.2f} segundos")

    rclpy.shutdown()

if __name__ == '__main__':
    main()
