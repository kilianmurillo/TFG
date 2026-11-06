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
        self.gear_ratio = 32.0     # Relación de la reductora 32:1
        self.max_motor_w = 376.99  # Velocidad máxima del motor interno (rad/s)

    def local_to_wheel_velocities(self, vx, vy, w):
        # Cálculo de la velocidad lineal de cada rueda (m/s)
        v1 = vy + self.R * w
        v2 = -(math.sqrt(3) / 2.0) * vx - 0.5 * vy + self.R * w
        v3 = (math.sqrt(3) / 2.0) * vx - 0.5 * vy + self.R * w
        
        # 1. Velocidad angular exigida a la RUEDA (rad/s)
        w1_wheel = v1 / self.r
        w2_wheel = v2 / self.r
        w3_wheel = v3 / self.r

        # 2. Velocidad que se le exige al MOTOR (rad/s) multiplicando por la reductora
        w1_motor = w1_wheel * self.gear_ratio
        w2_motor = w2_wheel * self.gear_ratio
        w3_motor = w3_wheel * self.gear_ratio

        # 3. Saturación física del MOTOR (no puede girar a más de 376.99 rad/s)
        w1_motor = max(-self.max_motor_w, min(self.max_motor_w, w1_motor))
        w2_motor = max(-self.max_motor_w, min(self.max_motor_w, w2_motor))
        w3_motor = max(-self.max_motor_w, min(self.max_motor_w, w3_motor))
        
        # 4. Velocidad real final de la RUEDA (rad/s) tras la reducción física
        # Esto es lo que se envía a Gazebo/ros2_control
        w1_final = w1_motor / self.gear_ratio
        w2_final = w2_motor / self.gear_ratio
        w3_final = w3_motor / self.gear_ratio
        
        return w1_final, w2_final, w3_final

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