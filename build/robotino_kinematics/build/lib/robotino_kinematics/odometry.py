#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist
from tf2_ros import TransformBroadcaster
import math
import numpy as np
import matplotlib.pyplot as plt
import time
import threading
from rclpy.time import Time

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0; aj /= 2.0; ak /= 2.0
    ci = math.cos(ai); si = math.sin(ai)
    cj = math.cos(aj); sj = math.sin(aj)
    ck = math.cos(ak); sk = math.sin(ak)
    cc = ci*ck; cs = ci*sk
    sc = si*ck; ss = si*sk
    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss
    return q

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        self.r = 0.060  
        self.L = 0.1755 

        self.x = 0.0; self.y = 0.0; self.theta = 0.0
        self.last_time = None 
        self.start_sim_time = None 
        
        self.current_vx = 0.0; self.current_vy = 0.0; self.current_wz = 0.0
        self.joint_names = ['rueda_joint', 'izquierda_joint', 'derecha_joint']

        # Matrices de Cinemática (Tus matemáticas exactas e intactas)
        self.M = -1.0 * self.r * np.array([
            [0.0, 1.0 / math.sqrt(3), -1.0 / math.sqrt(3)],
            [-2.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0],
            [1.0 / (3.0 * self.L), 1.0 / (3.0 * self.L), 1.0 / (3.0 * self.L)]
        ])
        self.M_inv = np.linalg.inv(self.M)

        # Candado (Lock) para evitar que el hilo de dibujo choque con el de ROS
        self.lock = threading.Lock()
        
        self.time_data = []
        self.cmd_data = [[], [], []]
        self.real_data = [[], [], []]
        self.current_cmd = [0.0, 0.0, 0.0]

        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("Nodo de ROS ejecutándose en hilo rápido (Threading activado).")

    def cmd_callback(self, msg):
        # Calculamos la consigna exacta y la guardamos instantáneamente bajo llave
        v = np.array([msg.linear.x, msg.linear.y, msg.angular.z])
        w = self.M_inv.dot(v)
        with self.lock:
            self.current_cmd = w.tolist()

    def wheel_to_local_velocities(self, w1, w2, w3, msg_stamp):
        current_time = Time.from_msg(msg_stamp)
        if self.last_time is None:
            self.last_time = current_time
            return self.x, self.y, self.theta

        dt = (current_time.nanoseconds - self.last_time.nanoseconds) / 1e9
        self.last_time = current_time

        if dt <= 0.0: return self.x, self.y, self.theta

        vx = -1.0 * self.r * ( 0.0 * w1 + (1.0 / math.sqrt(3)) * w2 - (1.0 / math.sqrt(3)) * w3 )
        vy = -1.0 * self.r * ( (-2.0 / 3.0) * w1 + (1.0 / 3.0) * w2 + (1.0 / 3.0) * w3 )
        wz = -1.0 * self.r * ( (1.0 / (3.0 * self.L)) * w1 + (1.0 / (3.0 * self.L)) * w2 + (1.0 / (3.0 * self.L)) * w3 )

        self.x += (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
        self.y += (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt
        self.theta += wz * dt

        self.current_vx = vx; self.current_vy = vy; self.current_wz = wz
        return self.x, self.y, self.theta

    def joint_state_callback(self, msg):
        try:
            w1 = msg.velocity[msg.name.index(self.joint_names[0])]
            w2 = msg.velocity[msg.name.index(self.joint_names[1])]
            w3 = msg.velocity[msg.name.index(self.joint_names[2])]
        except ValueError:
            return

        self.wheel_to_local_velocities(w1, w2, w3, msg.header.stamp)
        self.publish_odometry(msg.header.stamp)

        current_msg_time = Time.from_msg(msg.header.stamp)
        if self.start_sim_time is None:
            self.start_sim_time = current_msg_time
            
        current_t = (current_msg_time.nanoseconds - self.start_sim_time.nanoseconds) / 1e9
        
        # Guardamos los datos asegurando que matplotlib no interfiera
        with self.lock:
            self.time_data.append(current_t)
            for i in range(3):
                self.cmd_data[i].append(self.current_cmd[i])
            self.real_data[0].append(w1)
            self.real_data[1].append(w2)
            self.real_data[2].append(w3)

            if len(self.time_data) > 100:
                self.time_data.pop(0)
                for i in range(3):
                    self.cmd_data[i].pop(0)
                    self.real_data[i].pop(0)

    def publish_odometry(self, stamp):
        q = quaternion_from_euler(0, 0, self.theta)

        t = TransformStamped()
        t.header.stamp = stamp; t.header.frame_id = 'odom'; t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x; t.transform.translation.y = self.y; t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]; t.transform.rotation.y = q[1]; t.transform.rotation.z = q[2]; t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = stamp; odom.header.frame_id = 'odom'; odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x; odom.pose.pose.position.y = self.y; odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q[0]; odom.pose.pose.orientation.y = q[1]; odom.pose.pose.orientation.z = q[2]; odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = self.current_vx; odom.twist.twist.linear.y = self.current_vy; odom.twist.twist.angular.z = self.current_wz
        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    
    # 1. Ejecutamos ROS 2 en un hilo de fondo. 
    # Esto garantiza latencia CERO al procesar cmd_vel y joint_states.
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    # 2. El hilo principal se queda exclusivamente dibujando
    plt.ion()
    fig, ax = plt.subplots(3, 1, figsize=(8, 9))
    fig.suptitle("Respuesta de Velocidad: Consigna vs Real")
    fig.tight_layout(pad=3.0)

    nombres_ruedas = ["Rueda 1 (Trasera)", "Rueda 2 (Front-Der)", "Rueda 3 (Front-Izq)"]
    line_cmd = []; line_real = []

    for i in range(3):
        line_c, = ax[i].plot([], [], 'r--', label='Consigna')
        line_r, = ax[i].plot([], [], 'b-', label='Real')
        line_cmd.append(line_c)
        line_real.append(line_r)
        ax[i].set_title(nombres_ruedas[i])
        ax[i].set_ylabel("rad/s")
        ax[i].legend(loc="upper right")
        ax[i].grid(True)
    
    ax[2].set_xlabel("Tiempo Simulación (s)")

    try:
        while rclpy.ok():
            # Extraemos una copia rápida y segura de los datos
            with node.lock:
                t_data = list(node.time_data)
                c_data = [list(node.cmd_data[i]) for i in range(3)]
                r_data = [list(node.real_data[i]) for i in range(3)]
            
            # Dibujamos sin bloquear el motor interno de ROS
            if t_data:
                for i in range(3):
                    line_cmd[i].set_data(t_data, c_data[i])
                    line_real[i].set_data(t_data, r_data[i])
                    ax[i].relim()
                    ax[i].autoscale_view()
                fig.canvas.draw()
                fig.canvas.flush_events()
            
            time.sleep(0.05) # Refresco estable a 20Hz
            
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        plt.close('all')

if __name__ == '__main__':
    main()