# Clifford

Clifford es un proyecto de robot cuadrúpedo desarrollado sobre ROS 2. El repositorio contiene el modelo del robot, nodos de locomoción, cinemática, generación de trayectoria, visualización en RViz, simulación en Gazebo y comunicación serial hacia un microcontrolador.

Este proyecto fue desarrollado y probado en ROS 2 Humble.

## Entorno de Desarrollo

- Sistema operativo: Ubuntu con ROS 2 Humble.
- ROS: ROS 2 Humble Hawksbill.
- Sistema de compilación: `colcon`.
- Lenguaje principal: Python 3.
- Visualización: RViz 2.
- Simulación: Gazebo Classic mediante `gazebo_ros`.
- Control: `ros2_control`, `controller_manager` y `joint_trajectory_controller`.
- Comunicación externa: puerto serial con `pyserial`, por defecto `/dev/ttyUSB0`.

Antes de usar el workspace se debe cargar ROS 2:

```bash
source /opt/ros/humble/setup.bash
```

Después de compilar el workspace, se debe cargar el entorno local:

```bash
source install/setup.bash
```

## Estructura del Workspace

El workspace esperado tiene dos paquetes dentro de `src`:

```text
clifford_ws/
├── build/
├── install/
├── log/
└── src/
    ├── clifford/
    └── custom_interfaces/
```

### Paquete `clifford`

Contiene la lógica principal del robot:

```text
clifford/
├── clifford/
│   ├── inverse_kinematic.py
│   ├── locomotion.py
│   ├── numeric_method.py
│   ├── ros2arduino.py
│   ├── set_initial_pose.py
│   └── trajectory_service.py
├── config/
│   └── joint_group_trajectory_controller.yaml
├── description/
│   ├── clifford.urdf.xacro
│   ├── clifford_core.xacro
│   ├── clifford_core.sdf
│   └── model.config
├── launch/
│   ├── controller_manager.launch.py
│   ├── full_simulation.launch.py
│   ├── principal_mode.launch.py
│   ├── sim_gazebo_clifford.launch.py
│   └── visual_clifford.launch.py
├── meshes/
├── my_robot_config/
├── worlds/
├── package.xml
└── setup.py
```

### Paquete `custom_interfaces`

Contiene la interfaz de servicio usada por la trayectoria:

```text
custom_interfaces/
├── CMakeLists.txt
├── package.xml
└── srv/
    └── TrajectoryPoint.srv
```

El servicio `TrajectoryPoint` recibe un índice y un tipo de pata, y responde un punto cartesiano:

```text
int32 index
string leg_type
---
geometry_msgs/Point point
```

Campos:

- `index`: índice del punto de trayectoria solicitado.
- `leg_type`: tipo de pata. Actualmente se usa `front` para patas delanteras y `back` para patas traseras.
- `point`: punto `x, y, z` generado para la trayectoria.

## Dependencias Principales

Paquetes ROS usados por el proyecto:

- `rclpy`
- `geometry_msgs`
- `sensor_msgs`
- `trajectory_msgs`
- `nav_msgs`
- `tf2_ros`
- `robot_state_publisher`
- `joint_state_publisher_gui`
- `rviz2`
- `xacro`
- `gazebo_ros`
- `controller_manager`
- `joint_state_broadcaster`
- `joint_trajectory_controller`
- `rosidl_default_generators`
- `rosidl_default_runtime`

Dependencias Python usadas por los nodos:

- `numpy`
- `pandas`
- `pyserial`
- `setuptools`

Ejemplo de instalación de dependencias ROS:

```bash
sudo apt update
sudo apt install \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-rviz2 \
  ros-humble-xacro \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-controller-manager \
  ros-humble-joint-state-broadcaster \
  ros-humble-joint-trajectory-controller
```

Ejemplo de instalación de dependencias Python:

```bash
pip3 install numpy pandas pyserial
```

## Compilación

Desde la raíz del workspace:

```bash
cd ~/clifford_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

Si existen ejecutables antiguos en `install/` después de cambios grandes en `setup.py`, se recomienda hacer una compilación limpia:

```bash
cd ~/clifford_ws
rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## Lógica General del Sistema

El flujo principal del proyecto es:

1. El modelo del robot se carga desde Xacro/URDF para RViz o desde SDF para Gazebo.
2. `trajectory_service.py` genera puntos de trayectoria para las patas.
3. `locomotion.py` solicita puntos al servicio `get_trajectory_point`.
4. `numeric_method.py` calcula los ángulos articulares necesarios para alcanzar cada punto.
5. `locomotion.py` publica los ángulos en `/joint_states`.
6. `robot_state_publisher` usa `/joint_states` y `robot_description` para actualizar el modelo del robot en RViz.
7. Opcionalmente, `ros2arduino.py` escucha `/joint_states`, filtra los ángulos y los envía por serial al microcontrolador.

## Nodos Principales

### `trajectory_service.py`

Nodo ROS:

```text
/trajectory_service
```

Ejecutable:

```bash
ros2 run clifford trajectory_node_srv
```

Función:

- Crea el servicio `/get_trajectory_point`.
- Genera una trayectoria elíptica para patas delanteras y traseras.
- Devuelve un `geometry_msgs/Point`.

Servicio:

```text
/get_trajectory_point
```

Prueba manual:

```bash
ros2 service call /get_trajectory_point custom_interfaces/srv/TrajectoryPoint "{index: 0, leg_type: front}"
```

Respuesta esperada para el primer punto:

```text
point:
  x: -0.0
  y: 0.0
  z: -0.162
```

### `locomotion.py`

Nodo ROS:

```text
/quadruped_locomotion
```

Ejecutable:

```bash
ros2 run clifford locomotion_node
```

Función:

- Espera el servicio `/get_trajectory_point`.
- Solicita puntos de trayectoria.
- Calcula ángulos articulares usando `nodeNumericMethod`.
- Publica estados articulares en `/joint_states`.
- Publica una transformación TF entre `base_footprint` y `base_link`.

Publica:

```text
/joint_states
/tf
/odom
```

Consume:

```text
/get_trajectory_point
```

### `numeric_method.py`

Contiene el método numérico de cinemática inversa usado por la locomoción. Calcula los ángulos de las articulaciones a partir de un punto deseado.

El método trabaja con parámetros geométricos internos de la pata y usa una aproximación basada en Jacobiano/pseudoinversa y un ajuste iterativo.

### `inverse_kinematic.py`

Contiene una implementación geométrica alternativa de cinemática inversa. Calcula ángulos a partir de longitudes de eslabones y un punto deseado.

Ejecutable:

```bash
ros2 run clifford inverse_node
```

### `set_initial_pose.py`

Nodo ROS:

```text
/initial_pose_publisher
```

Ejecutable:

```bash
ros2 run clifford set_pose_node
```

Función:

- Publica una posición inicial en:

```text
/joint_trajectory_controller/joint_trajectory
```

### `ros2arduino.py`

Nodo ROS:

```text
/transform_angles_ros2arduino
```

Ejecutable:

```bash
ros2 run clifford ros2arduino_node
```

Función:

- Se suscribe a `/joint_states`.
- Convierte los ángulos de radianes a grados.
- Filtra ciertos valores válidos de articulaciones.
- Envía una lista de ángulos al microcontrolador por serial.

Puerto serial configurado:

```text
/dev/ttyUSB0
```

Baudrate:

```text
115200
```

Formato enviado:

```text
[angulo1,angulo2,angulo3,...]
```

Antes de ejecutar este nodo, conectar el microcontrolador y verificar el puerto:

```bash
ls /dev/ttyUSB*
ls /dev/ttyACM*
```

Si el puerto no existe, el nodo falla con un error similar a:

```text
could not open port /dev/ttyUSB0
```

Si hay problemas de permisos:

```bash
sudo usermod -a -G dialout $USER
```

Luego cerrar sesión y volver a entrar.

## Launch Files

### Visualización en RViz

Launch:

```bash
ros2 launch clifford visual_clifford.launch.py
```

Este launch inicia:

- `robot_state_publisher`
- `joint_state_publisher_gui`
- `rviz2`

Carga:

- Modelo URDF/Xacro: `description/clifford.urdf.xacro`
- Configuración RViz: `my_robot_config/my_robot_config.rviz`

Uso típico para ver el robot y mover articulaciones manualmente:

```bash
cd ~/clifford_ws
source install/setup.bash
ros2 launch clifford visual_clifford.launch.py
```

### RViz + Trayectoria

Para ver el robot en RViz y ejecutar la locomoción generada por trayectoria, usar tres terminales.

Terminal 1, RViz:

```bash
cd ~/clifford_ws
source install/setup.bash
ros2 launch clifford visual_clifford.launch.py
```

Terminal 2, servicio de trayectoria:

```bash
cd ~/clifford_ws
source install/setup.bash
ros2 run clifford trajectory_node_srv
```

Terminal 3, locomoción:

```bash
cd ~/clifford_ws
source install/setup.bash
ros2 run clifford locomotion_node
```

Verificación:

```bash
ros2 node info /quadruped_locomotion
```

Se espera ver publicaciones en:

```text
/joint_states
/tf
/odom
```

### Simulación en Gazebo

Launch:

```bash
ros2 launch clifford sim_gazebo_clifford.launch.py
```

Este launch:

- Abre Gazebo.
- Carga el mundo `worlds/empty.world`.
- Inserta el robot usando `description/clifford_core.sdf`.

### Controladores

Launch:

```bash
ros2 launch clifford controller_manager.launch.py
```

Este launch inicia:

```text
controller_manager / ros2_control_node
```

Usa la configuración:

```text
config/joint_group_trajectory_controller.yaml
```

Controladores configurados:

- `joint_state_broadcaster`
- `joint_trajectory_controller`

### Simulación Completa

Launch:

```bash
ros2 launch clifford full_simulation.launch.py
```

Este launch incluye:

- Gazebo con el robot.
- `controller_manager`.
- `set_pose_node`.

Actualmente la locomoción está comentada dentro de este launch, por lo que el nodo `locomotion_node` se puede ejecutar aparte si se desea mover el robot.

### Launch Principal

Launch:

```bash
ros2 launch clifford principal_mode.launch.py
```

Este launch está pensado para integrar:

- Nodo de locomoción.
- Nodo de trayectoria.
- Nodo de comunicación serial con Arduino.

Tiene en cuenta el modo de operación mediante el parámetro:

```bash
ros2 launch clifford principal_mode.launch.py modo:=home
```

Nota importante: en el estado actual del workspace pueden existir ejecutables antiguos instalados como `locomotion_node_2` o `trayectory_node`. Si estos fallan con `StopIteration`, significa que quedaron wrappers antiguos en `install/` y no tienen una entrada válida en el `setup.py` actual. En ese caso se recomienda limpiar y recompilar:

```bash
cd ~/clifford_ws
rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## Articulaciones del Robot

El robot usa 16 articulaciones principales:

```text
hombro_DA_joint
brazo_DA_joint
muneca_DA_joint
end_effector_DA_joint
hombro_IA_joint
brazo_IA_joint
muneca_IA_joint
end_effector_IA_joint
hombro_DT_joint
brazo_DT_joint
muneca_DT_joint
end_effector_DT_joint
hombro_IT_joint
brazo_IT_joint
muneca_IT_joint
end_effector_IT_joint
```

Convención de nombres:

- `DA`: derecha adelante.
- `IA`: izquierda adelante.
- `DT`: derecha trasera.
- `IT`: izquierda trasera.

## Modelo del Robot

Archivos principales:

- `description/clifford.urdf.xacro`: archivo Xacro principal para RViz.
- `description/clifford_core.xacro`: descripción completa del robot en formato Xacro.
- `description/clifford_core.sdf`: modelo usado por Gazebo.
- `description/model.config`: metadatos del modelo para Gazebo.
- `meshes/*.STL`: mallas 3D del robot.

El archivo `clifford.urdf.xacro` incluye `clifford_core.xacro`.

## Configuración de RViz

La configuración usada por el launch de visualización se encuentra en:

```text
my_robot_config/my_robot_config.rviz
```

Si RViz se abre sin mostrar el robot, revisar:

```bash
ros2 topic echo /robot_description --once
ros2 topic echo /joint_states --once
ros2 run tf2_tools view_frames
```

## Problema Conocido con VS Code Snap y RViz

Si se ejecuta desde una terminal integrada de VS Code instalado como Snap, RViz puede fallar con un error parecido a:

```text
symbol lookup error: /snap/core20/current/lib/x86_64-linux-gnu/libpthread.so.0:
undefined symbol: __libc_pthread_init, version GLIBC_PRIVATE
```

Esto no es un error del paquete `clifford`; ocurre por variables de entorno heredadas de Snap.

Una forma de lanzar RViz limpiando esas variables es:

```bash
cd ~/clifford_ws
env -u GTK_PATH \
    -u LOCPATH \
    -u SNAP \
    -u SNAP_NAME \
    -u SNAP_REVISION \
    -u SNAP_VERSION \
    -u SNAP_ARCH \
    -u SNAP_INSTANCE_NAME \
    -u SNAP_LIBRARY_PATH \
    -u SNAP_USER_DATA \
    -u SNAP_USER_COMMON \
    -u SNAP_COMMON \
    -u SNAP_DATA \
    -u SNAP_CONTEXT \
    -u SNAP_COOKIE \
    -u SNAP_REAL_HOME \
    -u SNAP_UID \
    -u SNAP_EUID \
    QT_QPA_PLATFORM=xcb \
    bash -lc 'source install/setup.bash && ros2 launch clifford visual_clifford.launch.py'
```

También se puede ejecutar desde una terminal normal del sistema en lugar de la terminal integrada de VS Code Snap.

## Comandos Útiles

Listar nodos:

```bash
ros2 node list
```

Ver información del nodo de locomoción:

```bash
ros2 node info /quadruped_locomotion
```

Listar tópicos:

```bash
ros2 topic list
```

Ver estados articulares:

```bash
ros2 topic echo /joint_states
```

Ver transformaciones:

```bash
ros2 topic echo /tf
```

Listar servicios:

```bash
ros2 service list
```

Probar el servicio de trayectoria:

```bash
ros2 service call /get_trajectory_point custom_interfaces/srv/TrajectoryPoint "{index: 0, leg_type: front}"
```

Listar ejecutables del paquete:

```bash
ros2 pkg executables clifford
```

## Estado Actual Validado

En el estado actual del workspace se validó lo siguiente:

- `colcon build` compila correctamente los paquetes `clifford` y `custom_interfaces`.
- El servicio `trajectory_node_srv` responde correctamente.
- `locomotion_node` se conecta al servicio de trayectoria y publica ángulos en `/joint_states`.
- `visual_clifford.launch.py` carga el modelo en `robot_state_publisher` y abre RViz si el entorno gráfico no está contaminado por Snap.
- `full_simulation.launch.py` inicia Gazebo, `spawn_entity`, `ros2_control_node` y `set_pose_node`, aunque puede fallar si el entorno gráfico/OpenGL no está disponible correctamente.
- `ros2arduino_node` requiere que exista el puerto `/dev/ttyUSB0`.

## Documentación del Proyecto

La carpeta `RUP/` contiene documentación del proceso de desarrollo:

- Casos de uso.
- Diagrama conceptual.
- Diagrama de clases.
- Diagramas de secuencia.
- Requerimientos funcionales y no funcionales.
- Manual de usuario.
- Pruebas de integración.

## Notas para Publicar en GitHub

Antes de publicar, revisar:

- Que el paquete `custom_interfaces` esté incluido junto al paquete `clifford`.
- Que no se suban carpetas generadas como `build/`, `install/` y `log/`.
- Que no se suban archivos temporales o cachés de Python.
- Que el puerto serial `/dev/ttyUSB0` puede cambiar dependiendo del computador.
- Que los documentos `.docx` de `RUP/` pueden mantenerse si hacen parte de la entrega del trabajo de grado.

Un `.gitignore` recomendado para el workspace:

```gitignore
build/
install/
log/
__pycache__/
*.pyc
.pytest_cache/
.vscode/
*.xlsx
```

## Autores

Proyecto desarrollado como trabajo de grado por Jagger.
