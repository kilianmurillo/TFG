import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray  # NUEVO: Importamos el tipo de mensaje para el array de velocidades
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
            
        # --- PUBLICADOR (Paso 7) ---
        # Creamos el publicador que enviará los comandos a ros2_control.
        self.publisher = self.create_publisher(
            Float64MultiArray, 
            '/wheel_velocity_controller/commands', 
            10)
            
        # --- MECANISMO DE SEGURIDAD (Perro guardián / Watchdog) ---
        # Registramos el tiempo exacto en el que recibimos el último comando.
        # Lo inicializamos con el tiempo actual de arranque.
        self.last_msg_time = self.get_clock().now()
        
        # Tiempo máximo permitido sin recibir comandos (en segundos) antes de detener el robot.
        self.timeout_sec = 0.5 
        
        # Creamos un temporizador que se ejecuta automáticamente cada 0.1 segundos.
        # Su función es vigilar constantemente si ha pasado demasiado tiempo.
        self.timer = self.create_timer(0.1, self.check_timeout_callback)
        
        # --- PARÁMETROS DEL ROBOT ---
        self.R = 0.1755  # Distancia del centro a las ruedas (m)
        self.r = 0.060  # Radio de la rueda (m)

    def local_to_wheel_velocities(self, vx, vy, w):
        # Ecuaciones cinemáticas para calcular velocidad lineal (m/s)
        v1 = -vx + self.R * w
        v2 = 0.5 * vx - (math.sqrt(3) / 2.0) * vy + self.R * w
        v3 = 0.5 * vx + (math.sqrt(3) / 2.0) * vy + self.R * w
        
        # Transformación a velocidades angulares (rad/s)
        w1 = v1 / self.r
        w2 = v2 / self.r
        w3 = v3 / self.r
        
        return w1, w2, w3

    def cmd_vel_callback(self, msg):
        # 1. Actualizamos el reloj. Le decimos al mecanismo de seguridad que acabamos de recibir un comando, así que reseteamos la cuenta atrás.
        self.last_msg_time = self.get_clock().now()
        
        # 2. Extraemos las velocidades solicitadas
        v_x = msg.linear.x
        v_y = msg.linear.y
        w = msg.angular.z
        
        # 3. Calculamos la cinemática de las ruedas
        w1, w2, w3 = self.local_to_wheel_velocities(v_x, v_y, w)
        
        # 4. En lugar de imprimirlo por terminal, llamamos a una nueva función que publica estas velocidades a ros2_control
        self.publish_wheel_commands(w1, w2, w3)

    def check_timeout_callback(self):
        # Calculamos cuánto tiempo ha pasado desde el último mensaje válido
        elapsed_time = self.get_clock().now() - self.last_msg_time
        
        # Si el tiempo transcurrido supera nuestro límite de 0.5 segundos:
        if elapsed_time.nanoseconds / 1e9 > self.timeout_sec:
            # Mandamos un comando de velocidad cero a las tres ruedas para frenar.
            self.publish_wheel_commands(0.0, 0.0, 0.0)

    def publish_wheel_commands(self, w1, w2, w3):
        # Creamos la variable que contendrá el mensaje
        msg = Float64MultiArray()
        
        # Asignamos las velocidades. 
        # IMPORTANTE: Mantenemos el orden exacto de ros2_control: 
        # 1. rueda_joint (w1), 2. izquierda_joint (w2), 3. derecha_joint (w3)
        msg.data = [w1, w2, w3]
        
        # Publicamos el mensaje
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