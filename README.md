# Proyecto_VNT
## Descripción del enfoque utilizado
EL controlador Follow the gap es una tecnica de navegacion basada en datos de un sensor LiDAR. EL objetivo es encontrar el gap mas grande en el entorno cercano para no colisionar.
### Como funciona:
- Se recibe una nube de puntos del LiDAR con el tòpico de /scan
- Se filtra rangos invalidos o muy cercanos que podrian afectar los giros del auto
- Se identifican gaps y se elige el mas amplio
- Se publica el estado de velocidad y angulo en el tòpico /drive
## Estructura del código
El proyecto cuenta con 2 archivos py
### Archivo followgap.py
Tiene su clase FollowGap que se suscribe al tòpico de /scan, ademas de ser un publicado de /drive.
Se define el mètodo lidar_callback que procesa los rangos que seran validos para seleccionar el gap, debido a que el auto tendra una burbujar invisible para mayuor precision de giros. Cuando se elige el mejor gap, se calcula la distancia media para calcular la velocidad a la que deberia ir el auto para no colisionar. Por ultimo, se crean y publican los cambios al tòpico de /drive.
### Archivo num_vueltas.py
Tiene su clase Lap_timer que se suscribe al tòpico /ego_racecar/odom y define parametros que sera la linea de meta.
Se define el mètodo de odom_callback donde lee cada momento mediante odometrìa la posicion del auto para que cada vez que pase se aumente el contador de vuelta y se reinicie el contador del tiempo.
## Instrucciones de ejecución
Pasos para ejecutar el codigo:
- Guardar los scripts dentro del workspace de ROS2
- Darle permiso de ejecucion con el comando "chmod +x follow_the_gap.py lap_timer.py"
- Asegurarse que los scripts estan en setup.py como:
entry_points={
    'console_scripts': [
        'follow_the_gap = your_package.follow_the_gap:main',
        'lap_timer = your_package.lap_timer:main',
    ],
- Entrar al directorio de Proyecto_VNT
- Construir el workspace con los comandos de "colcon build" y "source install/setup.bash"
- Lanzar el simulador de F1tenth con ros2 con el comando "ros2 launch f1tenth_gym_ros gym_bridge_launch.py"
- Ejecutar ambos scripts con los comandos de "ros2 run controllers followgap" y "ros2 run controllers num_vueltas"
