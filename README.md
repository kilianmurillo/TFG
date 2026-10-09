# TFG




## Tarea pendiente: Corrección de `display.launch.py`

- [ ] **Revisar `display.launch.py`:** Eliminar los elementos relacionados con Gazebo (`gazebo`, `spawn_robot`, `bridge` y `set_model_path`), junto con las importaciones (librerías) que ya no se utilicen.


## Próximas tareas: Implementación del modelo cinemático

**Objetivo:** Transformar las velocidades locales del Robotino \((v_x, v_y, \omega)\) en velocidades angulares de sus tres ruedas e integrarlas con `ros2_control`.

### Tareas pendientes

- **1. Crear el paquete ROS 2:** Crear `robotino_kinematics` con `ament_python` y eliminar la carpeta `test`.

- **2. Crear el nodo:** Implementar `local_to_wheel_velocities_node.py` con `rclpy`, mostrar un mensaje por terminal y comprobar su ejecución con `ros2 run`.

- **3. Suscribirse a `/cmd_vel`:** Recibir mensajes `geometry_msgs/msg/Twist` e imprimir \(v_x\), \(v_y\) y \(\omega\). Comprobarlo con `ros2 topic pub`.

- **4. Estudiar el modelo cinemático:** Revisar las ecuaciones (5)–(10) del [artículo](https://www.mdpi.com/2218-6581/14/6/72), identificar los parámetros geométricos y comprobar la correspondencia de las ruedas con el URDF.

- **5. Implementar la transformación:** Crear `local_to_wheel_velocities(vx, vy, w)` dentro del nodo para calcular e imprimir las velocidades angulares de las tres ruedas en rad/s.

- **6. Validar los cálculos:** Enviar comandos con `ros2 topic pub` (avance, retroceso, movimiento lateral, rotación y combinado) y comparar los resultados con las ecuaciones del artículo.

- **7. Integrar con `ros2_control`:** Publicar las velocidades en `/wheel_velocity_controller/commands` (`std_msgs/msg/Float64MultiArray`), respetando el orden de las ruedas y añadiendo un timeout de seguridad.

- **8. Validar en Gazebo:** Ejecutar `sim.launch.py`, el nodo con `ros2 run` y `teleop_twist_keyboard` para comprobar el movimiento del Robotino.

