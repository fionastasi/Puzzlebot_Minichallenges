# Plan de ArUco para Correccion de Odometria

## Estado actual

Por ahora el paquete solo integra deteccion de ArUcos en el launch fisico de Bug2.
No se modifica la odometria ni se cambia el comportamiento de navegacion.

El launch arranca el detector existente:

```bash
ros2 run aruco_opencv aruco_tracker_autostart --ros-args -p cam_base_topic:=/image_raw -p marker_size:=0.06
```

Y tambien arranca `aruco_detection_monitor`, un nodo propio que escucha detecciones y reporta el marcador mas cercano.

Parametros relevantes del launch:

```bash
use_aruco_tracker:=true
use_aruco_monitor:=true
aruco_cam_base_topic:=/image_raw
aruco_marker_size:=0.06
aruco_detection_topic:=/marker_publisher/markers_list
aruco_detection_type:=markers_list
```

Para imprimir solo IDs y coordenadas conocidas no se necesita `aruco_msgs`.
El monitor escucha `/marker_publisher/markers_list` como `std_msgs/Int32MultiArray`.

Si se quiere escuchar `/marker_publisher/markers` para obtener distancia/pose relativa, la maquina que corre el monitor debe tener instalado/sourceado `aruco_msgs`.
Entonces usar:

```bash
aruco_detection_topic:=/marker_publisher/markers
aruco_detection_type:=aruco_msgs
```

## Datos fisicos asumidos

- Los marcadores estan pegados en paredes verticales del laberinto.
- Cada marcador individual mide `9.5 cm x 9.5 cm`.
- El comando actual del detector usa `marker_size:=0.06`.
- La camara esta fija al robot, centrada al frente.
- Robot aproximado:
  - ancho: `0.20 m`
  - largo: `0.23 m`
  - camara elevada: `0.13 m`
- El sistema de coordenadas de los ArUcos sera el mismo que usan las metas de Bug2.

Nota: conviene confirmar si `marker_size` debe ser `0.06` o `0.095`.
En detectores ArUco normalmente `marker_size` es el lado interno negro/codificado del marcador, no necesariamente el papel completo.

## Seleccion de marcador

Si se detectan varios ArUcos al mismo tiempo, se usara el mas cercano a la camara.
La distancia se calcula con la norma de la posicion relativa:

```text
dist = sqrt(x^2 + y^2 + z^2)
```

Esto evita corregir contra marcadores mas lejanos o con peor estimacion de pose.

## Mapa de marcadores propuesto

La siguiente fase debe agregar una tabla interna o YAML con IDs y posiciones globales:

```yaml
markers:
  70:
    x: 1.84
    y: -0.30
  705:
    x: 0.90
    y: -1.20
  706:
    x: 2.39
    y: -1.26
  708:
    x: 1.19
    y: -1.21
  703:
    x: 1.23
    y: -2.07
  702:
    x: 0.28
    y: -1.82
  75:
    x: 2.74
    y: -2.40
  701:
    x: 2.84
    y: 0.00
```

`theta` representa la orientacion del marcador en el marco global del laberinto.
Como los marcadores estan en paredes, esta orientacion es necesaria para convertir la pose relativa vista por la camara en pose global del robot.

Nota: el ID `706` fue registrado como `2.39, -126`; se asumio que la coordenada correcta es `2.39, -1.26`.

## Correccion futura de odometria

La relacion de transforms sera:

```text
T_map_base = T_map_marker * inverse(T_base_marker)
T_base_marker = T_base_camera * T_camera_marker
```

Donde:

- `T_map_marker`: pose global conocida del ArUco.
- `T_camera_marker`: pose relativa detectada por la camara.
- `T_base_camera`: offset fijo de la camara respecto al centro del robot.
- `T_map_base`: pose global estimada del robot.

La correccion no deberia ser brusca al inicio. Recomendacion inicial:

```text
pose_corregida = 0.8 * pose_odom + 0.2 * pose_aruco
```

Se puede aumentar el peso del ArUco cuando:

- el marcador esta cerca,
- el ID esta en la tabla conocida,
- la deteccion es estable por varios frames,
- el angulo de vision no es extremo.

## Arquitectura recomendada

Mantener Bug2 separado de percepcion:

```text
aruco_opencv -> aruco_detection_monitor -> aruco_odom_corrector -> /odom_corrected -> bug2_node
```

Fases:

1. `aruco_detection_monitor`: solo imprime ID, distancia y pose relativa del marcador cercano.
2. `aruco_odom_corrector`: estima pose global del robot, pero solo la publica como diagnostico.
3. Publicar `/odom_corrected`.
4. Cambiar `challengeBug2_physical.launch.py` para que Bug2 use `odom_topic:=odom_corrected`.

## Comandos de prueba

Launch completo con ArUco:

```bash
ros2 launch minichallenge6_new_test challengeBug2_physical.launch.py
```

Solo comprobar que el tracker publica:

```bash
ros2 topic list | grep -i aruco
ros2 topic list | grep marker
ros2 topic echo /marker_publisher/markers
```

Si el topico nativo existe:

```bash
ros2 topic echo /aruco_detections
```
