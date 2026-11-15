#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster
import math
import numpy as np
import matplotlib.pyplot as plt
import time
from rclpy.time import Time

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss
    return q

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        # Parámetros del robot
        self.r = 0.060  # Radio de la rueda (m)
        self.L = 0.1755 # Distancia del centro a la rueda (m)

        # Variables de estado y cinemática
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Inicializamos last_time a None para tomar el primer timestamp de Gazebo
        self.last_time = None 
        
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_wz = 0.0

        self.joint_names = ['rueda_joint', 'izquierda_joint', 'derecha_joint']

        # Almacenamiento de datos para la gráfica de 3 ruedas
        self.start_time = time.time()
        self.time_data = []
        self.cmd_data = [[], [], []]
        self.real_data = [[], [], []]
        self.current_cmd = [0.0, 0.0, 0.0]

        # Configuración de matplotlib (3 subgráficos)
        plt.ion()
        self.fig, self.ax = plt.subplots(3, 1, figsize=(8, 9))
        self.fig.suptitle("Respuesta de Velocidad: Consigna vs Real")
        self.fig.tight_layout(pad=3.0)

        # Configuración visual de cada rueda
        nombres_ruedas = ["Rueda 1 (Trasera)", "Rueda 2 (Front-Der)", "Rueda 3 (Front-Izq)"]
        self.line_cmd = []
        self.line_real = []

        for i in range(3):
            line_c, = self.ax[i].plot([], [], 'r--', label='Consigna')
            line_r, = self.ax[i].plot([], [], 'b-', label='Real')
            self.line_cmd.append(line_c)
            self.line_real.append(line_r)
            self.ax[i].set_title(nombres_ruedas[i])
            self.ax[i].set_ylabel("rad/s")
            self.ax[i].legend(loc="upper right")
            self.ax[i].grid(True)
        
        self.ax[2].set_xlabel("Tiempo (s)")

        # Suscriptores y Publicadores
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.cmd_sub = self.create_subscription(Float64MultiArray, '/wheel_velocity_controller/commands', self.cmd_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Temporizador para actualizar la gráfica
        self.plot_timer = self.create_timer(0.1, self.update_plot)
        self.get_logger().info("Nodo de odometría inicializado.")

    def cmd_callback(self, msg):
        if len(msg.data) >= 3:
            self.current_cmd = [msg.data[0], msg.data[1], msg.data[2]]

    def wheel_to_local_velocities(self, w1, w2, w3, msg_stamp):
        # Usamos el timestamp extraído del mensaje de Gazebo
        current_time = Time.from_msg(msg_stamp)
        
        if self.last_time is None:
            self.last_time = current_time
            return self.x, self.y, self.theta

        dt = (current_time.nanoseconds - self.last_time.nanoseconds) / 1e9
        self.last_time = current_time

        # Evitamos cálculos si el dt es 0 o negativo (previene saltos temporales)
        if dt <= 0.0:
            return self.x, self.y, self.theta

        # Matriz Jacobiana Invertida (-1)
        vx = -1.0 * self.r * ( 0.0 * w1 + (1.0 / math.sqrt(3)) * w2 - (1.0 / math.sqrt(3)) * w3 )
        vy = -1.0 * self.r * ( (-2.0 / 3.0) * w1 + (1.0 / 3.0) * w2 + (1.0 / 3.0) * w3 )
        wz = -1.0 * self.r * ( (1.0 / (3.0 * self.L)) * w1 + (1.0 / (3.0 * self.L)) * w2 + (1.0 / (3.0 * self.L)) * w3 )

        # Integración en el Marco Global
        delta_x = (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
        delta_y = (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt
        delta_theta = wz * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        self.current_vx = vx
        self.current_vy = vy
        self.current_wz = wz

        return self.x, self.y, self.theta

    def joint_state_callback(self, msg):
        try:
            w1 = msg.velocity[msg.name.index(self.joint_names[0])]
            w2 = msg.velocity[msg.name.index(self.joint_names[1])]
            w3 = msg.velocity[msg.name.index(self.joint_names[2])]
        except ValueError:
            return

        # Pasamos la marca de tiempo exacta del mensaje
        self.wheel_to_local_velocities(w1, w2, w3, msg.header.stamp)

        # Guardar datos para la gráfica
        current_t = time.time() - self.start_time
        self.time_data.append(current_t)
        
        for i in range(3):
            self.cmd_data[i].append(self.current_cmd[i])
        
        self.real_data[0].append(w1)
        self.real_data[1].append(w2)
        self.real_data[2].append(w3)

        # Limitar memoria intermedia
        if len(self.time_data) > 100:
            self.time_data.pop(0)
            for i in range(3):
                self.cmd_data[i].pop(0)
                self.real_data[i].pop(0)

        # Publicamos la odometría utilizando la misma marca de tiempo
        self.publish_odometry(msg.header.stamp)

    def update_plot(self):
        if not self.time_data:
            return

        for i in range(3):
            self.line_cmd[i].set_data(self.time_data, self.cmd_data[i])
            self.line_real[i].set_data(self.time_data, self.real_data[i])
            self.ax[i].relim()
            self.ax[i].autoscale_view()
            
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def publish_odometry(self, stamp):
        q = quaternion_from_euler(0, 0, self.theta)

        t = TransformStamped()
        t.header.stamp = stamp  # <-- ASIGNACIÓN DIRECTA DEL TIMESTAMP DEL SIMULADOR
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = stamp  # <-- ASIGNACIÓN DIRECTA
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        odom.twist.twist.linear.x = self.current_vx
        odom.twist.twist.linear.y = self.current_vy
        odom.twist.twist.angular.z = self.current_wz

        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        # Esto asegura que la ventana gráfica se cierre y no deje zombis
        plt.close('all') 

if __name__ == '__main__':
    main()