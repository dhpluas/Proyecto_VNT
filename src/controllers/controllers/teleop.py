import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

msg = """
Control del robot:
---------------------------
W/S: avanzar/retroceder
A/D: girar izquierda/derecha
Q: salir
---------------------------
"""

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.speed = 3.0      # velocidad lineal máxima
        self.turn = 2.0       # velocidad angular máxima

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def run(self):
        print(msg)
        while True:
            key = self.getKey()
            twist = Twist()
            if key == 'w':
                twist.linear.x = self.speed
            elif key == 's':
                twist.linear.x = -self.speed
            elif key == 'a':
                twist.angular.z = self.turn
            elif key == 'd':
                twist.angular.z = -self.turn
            elif key == 'q':
                print("Saliendo...")
                break
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            self.publisher.publish(twist)

if __name__ == '__main__':
    import termios
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = TeleopNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
