import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math

class LocalToWheelVelocitiesNode(Node):
    def __init__(self):
        super().__init__('local_to_wheel_velocities_node')
        self.get_logger().info('Nodo de cinemática inicializado. Esperando comandos...')
        
        # --- SUSCRIPTOR ---
        # Escuchamos las velocidades locales en /cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
            
        # --- PUBLICADOR ---
        # Enviamos los comandos calculados a ros2_control
        self.publisher = self.create_publisher(
            Float64MultiArray, 
            '/wheel_velocity_controller/commands', 
            10)
            
        # --- MECANISMO DE SEGURIDAD (Watchdog) ---
        self.last_msg_time = self.get_clock().now()
        self.timeout_sec = 0.5 
        self.timer = self.create_timer(0.1, self.check_timeout_callback)
        
        # --- PARÁMETROS DEL ROBOT ---
        self.R = 0.1755  # Distancia del centro a las ruedas (m)
        self.r = 0.060   # Radio de la rueda (m)

    def local_to_wheel_velocities(self, vx, vy, w):
        """
        Ecuaciones cinemáticas ajustadas al sentido de giro de las articulaciones del URDF:
        - Rueda 1 (rueda_joint, trasera): 180°
        - Rueda 2 (izquierda_joint, frontal derecha): -60°
        - Rueda 3 (derecha_joint, frontal izquierda): 60°
        """
        # Cálculo de la velocidad lineal de cada rueda (m/s)
        v1 = -vy + self.R * w
        v2 = -(math.sqrt(3) / 2.0) * vx + 0.5 * vy + self.R * w
        v3 = (math.sqrt(3) / 2.0) * vx + 0.5 * vy + self.R * w
        
        # Transformación a velocidades angulares (rad/s)
        w1 = v1 / self.r
        w2 = v2 / self.r
        w3 = v3 / self.r
        
        return w1, w2, w3

    def cmd_vel_callback(self, msg):
        # Actualizamos el reloj de seguridad
        self.last_msg_time = self.get_clock().now()
        
        # Extraemos velocidades solicitadas
        v_x = msg.linear.x
        v_y = msg.linear.y
        w = msg.angular.z
        
        # Calculamos la cinemática
        w1, w2, w3 = self.local_to_wheel_velocities(v_x, v_y, w)
        
        # Publicamos a ros2_control
        self.publish_wheel_commands(w1, w2, w3)

    def check_timeout_callback(self):
        # Frenar el robot si no recibe comandos en más de 0.5 segundos
        elapsed_time = self.get_clock().now() - self.last_msg_time
        if elapsed_time.nanoseconds / 1e9 > self.timeout_sec:
            self.publish_wheel_commands(0.0, 0.0, 0.0)

    def publish_wheel_commands(self, w1, w2, w3):
        msg = Float64MultiArray()
        # Orden exacto para ros2_control: [rueda_joint, izquierda_joint, derecha_joint]
        msg.data = [w1, w2, w3]
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LocalToWheelVelocitiesNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Deteniendo el nodo de cinemática...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()