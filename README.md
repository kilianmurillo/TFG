 TFG - Diseño y modelado del robot omnidireccional Robotino 4 para simulación en Gazebo

## Descripción del Proyecto
Diseño en SolidWorks del Robotino, un robot omnidireccional, con el objetivo de poder hacer una simulación realista en la última versión de Gazebo. El proyecto busca conseguir que el robot se pueda mover de manera teleoperada en simulación, estudiar con detalle el coeficiente de fricción de las ruedas omnidireccionales y modelar un buen controlador PID para cada una de las ruedas.

## Estructura y Función de los Paquetes
El proyecto está dividido en varios paquetes de ROS 2 para mantener el código organizado y modular:

*   **`robotino_description`**: Contiene la definición física e inercial del robot mediante archivos URDF y Xacro. Incluye las mallas 3D (`.STL`) exportadas desde SolidWorks para la visualización, así como colisiones optimizadas (cilindros) y los parámetros de fricción necesarios para la simulación física.
*   **`robotino_bringup`**: Contiene los archivos de lanzamiento (`launch`) y de configuración. Su función principal es iniciar la simulación en Gazebo, arrancar los controladores de hardware (`ros2_control`) y lanzar el visualizador RViz (por ejemplo, mediante `sim.launch.py` o `display.launch.py`).
*   **`robotino_kinematics`**: Paquete que incluye los scripts de control, destacando el nodo programado en Python (`local_to_wheel_velocities_node`). Su función es suscribirse a las órdenes de movimiento (`/cmd_vel`) y aplicar la matriz cinemática inversa para traducir esas velocidades generales en velocidades angulares individuales para cada una de las tres ruedas.

## Instrucciones de Ejecución

Sigue estos pasos en terminales separadas para ejecutar el proyecto desde cero:

**1. Compilar el espacio de trabajo**
Navega a la carpeta de tu proyecto y compila los paquetes:
```bash
cd ~/Rutadelws
colcon build
```

**2. Cargar el entorno y lanzar la simulación**
Carga las variables de ROS 2 y lanza Gazebo junto con RViz y los controladores:
```bash
source install/setup.bash
ros2 launch robotino_bringup sim.launch.py
```

**3. Iniciar el nodo de cinemática**
En una terminal nueva, carga el entorno y ejecuta el nodo que calcula el movimiento de las ruedas:
```bash
cd ~/Desktop/TFG
source install/setup.bash
ros2 run robotino_kinematics local_to_wheel_velocities_node
```

**4. Teleoperar el robot**
En una tercera terminal, lanza el nodo de control por teclado para enviar velocidades:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
*(Nota de uso: Mantén pulsada la tecla **Shift** (mayúsculas) para utilizar el modo holonómico y enviar comandos de desplazamiento lateral puro mediante las teclas J y L).*