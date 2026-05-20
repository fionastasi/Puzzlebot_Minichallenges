# Arquitectura — Bug0 (primera parte)

Resumen breve
- Nodo principal: `bug0` (FSM) — coordina comportamiento reactivo.
- Reutiliza nodos ya existentes:
  - `localisation` (publica `/odom`)
  - `kinematic_model` (simula / aplica `/cmd_vel`)
  - `display.launch.py` (robot_state_publisher + rviz2 + URDF_tfs)
  - módulos existentes: wall_following, go_to_goal (su lógica se integro conceptualmente en `bug0`)
- Sensores / tópicos:
  - `/scan` (LaserScan) — detección de obstáculos
  - `/odom` (Odometry) — pose estimada
  - `/cmd_vel` (Twist) — comandos de velocidad (publicados por `bug0`)

Estados (FSM)
- GO_TO_GOAL
  - Objetivo: mover al robot hacia la meta usando control proporcional en rho/alpha.
  - Comportamiento: rotar en sitio hasta alinear (|alpha| > umbral), luego avanzar en línea recta.
  - Transición → WALL_FOLLOWING si sensor frontal detecta obstáculo cercano.
- WALL_FOLLOWING
  - Objetivo: bordear el obstáculo de forma reactiva hasta recuperar línea de visión hacia la meta.
  - Comportamiento: control simple que mantiene separación del obstáculo usando el punto más cercano en el sector frontal.
  - Transición → GO_TO_GOAL si la inspección del LiDAR en la dirección de la meta está libre (distancia > clear_distance).
- GOAL_REACHED
  - Objetivo: detener el robot cuando rho < goal_tolerance.

Matemáticas principales
- Rho (distancia euclidiana): rho = sqrt((x_goal - x)^2 + (y_goal - y)^2)
- Alpha (ángulo a la meta en frame robot): alpha = normalize(atan2(dy, dx) - theta)
- Control:
  - Rotación: w = clamp(k_alpha * alpha, -w_max, w_max)
  - Traslación: v = clamp(k_rho * rho, -v_max, v_max) (solo si |alpha| < threshold)
- Normalización de ángulos: normalize(a) = atan2(sin(a), cos(a))

Detección de obstáculos / criterios
- `obstacle_blocking_path()`:
  - Revisa el sector frontal ±front_angle en `scan`.
  - Si min_range en ese sector < `obstacle_distance` → obstáculo que bloquea camino directo.
- `is_path_clear_to_goal()`:
  - Calcula ángulo al objetivo en frame robot.
  - Examina ventana angular alrededor de ese ángulo en `scan`.
  - Si min_range > `clear_distance` → camino despejado.

Publicadores / Subscriptores / Timers / QoS
- Publishers:
  - `/cmd_vel` (geometry_msgs/Twist) — QoS default 10 (reliable)
- Subscribers:
  - `/odom` (nav_msgs/Odometry) — QoS 10
  - `/scan` (sensor_msgs/LaserScan) — QoS 10
- Timer:
  - `dt = 0.02 s` (50 Hz) para `control_loop` — sampling razonable para reacción.
- Nota: ajustar QoS según simulador si se usan latencias o comunicación inter-nodos.

Integración con módulos existentes
- No duplicar localización, cinemática ni RViz: `bug0` consume `/odom` y `/scan` y publica `/cmd_vel`.
- Si existe un nodo `wall_following` o `go_to_goal`, `bug0` puede lanzar o invocar sus funcionalidades por tópicos/servicios; en esta primera versión se incorpora comportamiento reactivo simple dentro de `bug0` reutilizando los tópicos estándar.

Diagrama lógico (texto)
1. start → estado GO_TO_GOAL
2. GO_TO_GOAL:
   - calcular rho, alpha
   - si obstáculo frontal → cambiar a WALL_FOLLOWING
   - else: rotar (si |alpha|>umbral) o avanzar
3. WALL_FOLLOWING:
   - seguir borde del obstáculo (control reactivo desde LiDAR)
   - si `is_path_clear_to_goal()` → cambiar a GO_TO_GOAL
4. GOAL_REACHED:
   - detener y mantener estado

Pruebas y escenario de validación
- Casos:
  - Escenario sin obstáculos → `bug0` debe llegar por GO_TO_GOAL.
  - Obstáculo directo en la línea al objetivo → `bug0` debe entrar en WALL_FOLLOWING y salir cuando la meta sea visible.
  - Objetivo alcanzado → robot detenido.

Comandos para compilar y lanzar (resumen)
1. Construir workspace:
   - cd /home/aaraizae/Puzzlebot_Minichallenges/MiniChallenge3
   - colcon build --packages-select puzzlebot_sim
   - source install/setup.bash
2. Lanzar nodo Bug0 con RViz (incluye robot_state_publisher):
   - ros2 launch puzzlebot_sim bug0.launch.py
   - (ó) ros2 launch puzzlebot_sim display.launch.py & ros2 run puzzlebot_sim bug0

Cómo arrancar Gazebo y visualizar el robot
- Lanzar Gazebo con un mundo (ejemplo genérico):
  - ros2 launch gazebo_ros gazebo.launch.py world:=/full/path/to/your.world
  - Si tu paquete incluye worlds: 
    - ros2 launch gazebo_ros gazebo.launch.py world:=$(ros2 pkg prefix puzzlebot_sim)/share/puzzlebot_sim/worlds/mi_mundo.world
- Spawn del robot URDF en Gazebo (si `display.launch.py` no lo hace):
  - ros2 run gazebo_ros spawn_entity.py -file /home/aaraizae/Puzzlebot_Minichallenges/MiniChallenge3/src/puzzlebot_sim/urdf/puzzlebot.urdf -entity puzzlebot
  - Alternativa usando robot_description:
    - ros2 run gazebo_ros spawn_entity.py -topic /robot_description -entity puzzlebot
- Lanzar localización y cinemática (si no incluidos):
  - ros2 run puzzlebot_sim kinematic_model
  - ros2 run puzzlebot_sim localisation
- Abrir RViz:
  - ros2 run rviz2 rviz2
  - En RViz: añadir `RobotModel` (topic: `robot_description`) y `LaserScan` (`/scan`), `Odometry` (`/odom`), `TF` activo.
- Para cargar un mapa (opcional, map server):
  - ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/path/to/map.yaml
  - En RViz añadir `Map` y seleccionar topic `/map`.

Consejos prácticos
- Verifica que haya un suscriptor a `/cmd_vel` (simulador o kinematic_model). `ros2 topic info /cmd_vel` debe mostrar subscription count ≥ 1.
- Ajusta `obstacle_distance`, `clear_distance` y `front_angle` según densidad del LiDAR y escala del mundo.
- Si se requiere el Bug0 clásico (hit-point/m-line), añadir registro de "hit point" y criterio geométrico de cruce de m-line.

Posibles mejoras (siguientes pasos)
- Implementar detector de "hit point" y "leave point" para salida más robusta del borde.
- Integrar `localisation.sigma` (covarianza) para adaptar velocidades/umbrales según incertidumbre.
- Sustituir wall-following reactivo por el módulo `wall_following` ya existente (si expone servicio o tópico de control).
- Registrar métricas (tiempo, distancia recorrida, número de switches de estado).

---

Archivo `bug0.py` y `bug0.launch.py` ya están en el repositorio; para pruebas rápidas ejecuta los comandos de compilación y `ros2 launch puzzlebot_sim bug0.launch.py`.  
