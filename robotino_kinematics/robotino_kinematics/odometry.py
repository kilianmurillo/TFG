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
import threading
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
        self.last_time = None
        self.start_sim_time = None

        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_wz = 0.0

        self.joint_names = ['rueda_joint', 'izquierda_joint', 'derecha_joint']

        # Protección para compartir datos entre el hilo de ROS y el de matplotlib
        self.lock = threading.Lock()
        self.time_data = []
        self.cmd_data = [[], [], []]
        self.real_data = [[], [], []]
        self.current_cmd = [0.0, 0.0, 0.0]

        # depth=1 para leer siempre el último dato sin acumular retraso en cola
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 1
        )
        self.cmd_sub = self.create_subscription(
            Float64MultiArray, '/wheel_velocity_controller/commands', self.cmd_callback, 1
        )
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("Nodo de odometría inicializado (hilo separado + depth=1).")

    def cmd_callback(self, msg):
        if len(msg.data) >= 3:
            with self.lock:
                self.current_cmd = [msg.data[0], msg.data[1], msg.data[2]]

    def wheel_to_local_velocities(self, w1, w2, w3, msg_stamp):
        current_time = Time.from_msg(msg_stamp)

        if self.last_time is None:
            self.last_time = current_time
            return self.x, self.y, self.theta

        dt = (current_time.nanoseconds - self.last_time.nanoseconds) / 1e9
        self.last_time = current_time

        if dt <= 0.0:
            return self.x, self.y, self.theta

        # Matriz Jacobiana Invertida (-1)
        vx = -1.0 * self.r * ( 0.0 * w1 + (1.0 / math.sqrt(3)) * w2 - (1.0 / math.sqrt(3)) * w3 )
        vy = -1.0 * self.r * ( (-2.0 / 3.0) * w1 + (1.0 / 3.0) * w2 + (1.0 / 3.0) * w3 )
        wz = -1.0 * self.r * ( (1.0 / (3.0 * self.L)) * w1 + (1.0 / (3.0 * self.L)) * w2 + (1.0 / (3.0 * self.L)) * w3 )

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

        self.wheel_to_local_velocities(w1, w2, w3, msg.header.stamp)
        self.publish_odometry(msg.header.stamp)

        # Tiempo de simulación de Gazebo
        current_msg_time = Time.from_msg(msg.header.stamp)
        if self.start_sim_time is None:
            self.start_sim_time = current_msg_time
        current_t = (current_msg_time.nanoseconds - self.start_sim_time.nanoseconds) / 1e9

        with self.lock:
            self.time_data.append(current_t)
            for i in range(3):
                self.cmd_data[i].append(self.current_cmd[i])

            self.real_data[0].append(w1)
            self.real_data[1].append(w2)
            self.real_data[2].append(w3)

            # Mantiene siempre los últimos 15 segundos en pantalla
            while self.time_data and (current_t - self.time_data[0]) > 15.0:
                self.time_data.pop(0)
                for i in range(3):
                    self.cmd_data[i].pop(0)
                    self.real_data[i].pop(0)

    def publish_odometry(self, stamp):
        q = quaternion_from_euler(0, 0, self.theta)

        t = TransformStamped()
        t.header.stamp = stamp
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
        odom.header.stamp = stamp
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

    # Hilo secundario dedicado exclusivamente a ROS 2
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    # Hilo principal dedicado exclusivamente a la gráfica
    plt.ion()
    fig, ax = plt.subplots(3, 1, figsize=(8, 9))
    fig.suptitle("Respuesta de Velocidad: Consigna vs Real")
    fig.tight_layout(pad=3.0)

    nombres_ruedas = ["Rueda 1 (Trasera)", "Rueda 2 (Front-Der)", "Rueda 3 (Front-Izq)"]
    line_cmd = []
    line_real = []

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
            with node.lock:
                t_data = list(node.time_data)
                c_data = [list(node.cmd_data[i]) for i in range(3)]
                r_data = [list(node.real_data[i]) for i in range(3)]

            if t_data:
                for i in range(3):
                    line_cmd[i].set_data(t_data, c_data[i])
                    line_real[i].set_data(t_data, r_data[i])
                    ax[i].relim()
                    ax[i].autoscale_view()
                fig.canvas.draw()
                fig.canvas.flush_events()

            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        plt.close('all')

if __name__ == '__main__':
    main()