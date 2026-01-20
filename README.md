# TurtleBot3: Navegación Autónoma con ROS 2 Foxy

Este repositorio contiene los scripts y configuraciones necesarias para simular un robot TurtleBot3 capaz de navegar y esquivar obstáculos utilizando **ROS 2 (versión Foxy)** y **Gazebo**.

El proyecto implementa un algoritmo de navegación autónoma (Q-Learning / Evasión de obstáculos) que permite al robot tomar decisiones basadas en la lectura de sus sensores láser (LiDAR).

---

## Requisitos Previos:

Para ejecutar este proyecto necesitas tener preinstalado el siguiente software:

* **Sistema Operativo:** Ubuntu 20.04 (Nativo o WSL).
* **Simulador:** Gazebo multi-robot simulator, version 11.15.1
* **Middleware:** ROS 2 (Distribución **Foxy Fitzroy**).

---

## Instalación y Configuración

Sigue estos pasos para preparar el entorno de simulación.

### Paso 1: Instalar paquetes de TurtleBot3
Instalamos los paquetes necesarios para la simulación y el control del robot en ROS 2 Foxy:

```bash
sudo apt install ros-foxy-turtlebot3 ros-foxy-turtlebot3-msgs ros-foxy-turtlebot3-simulations ros-foxy-dynamixel-sdk -y
```

A continuación, configuramos el modelo del robot (versión "Burger") en las variables de entorno para que se cargue siempre al iniciar la terminal:

```echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
```
### Paso 2: Lanzar el Escenario (Gazebo).
No utilizaremos el mundo vacío por defecto. Cargaremos el escenario oficial de TurtleBot3 (turtlebot3_world), que incluye columnas y obstáculos geométricos ideales para el entrenamiento.
Abriendo el términal, ejecuta:
```
# Asegúrate de tener el entorno de foxy cargado:
source /opt/ros/foxy/setup.bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
### Paso 3: Ejecutar el Script de Control
El "cerebro" del robot es un script en Python que procesa los datos del láser y envía comandos de velocidad.

Asegúrate de tener el archivo .py en tu carpeta (robot_aprendiz.py).

Abre una nueva terminal:
```
# Cargar entorno ROS 2
source /opt/ros/foxy/setup.bash
```
Para posteriormente ejecutar el script, archivo python: "robot_aprendiz.py" para poner en funcionamiento el robot:

1. Primero creamos el archivo python que vamos a utilizar:
```
#Creamos el archivo python
nano robot_aprendiz.py
```
2. Copiamos el código del programa, y lo guardamos (Ctrl+O).
3. Ejecutamos el código: 
```
# Ejecutar el código
python3 robot_aprendiz.py
```
Si queremos ejecturar el script automático, en este caso el hiperparámetro: Épsilon, si queremos comprobar las demás variables se modificaría el código, para que itere el hiperparámetro deseado, sigue los mismos pasos:
1. Creamos el archivo python
```
nano robot_aprendiz_epsilon_automatico.py
```
2. Copiamos el código del programa, y lo guardamos (Ctrl+O).
3. Ejecutamos el código:
```
pyhton3 robot_aprendiz_epsilon_automatico.py
```

Una vez realizado estos pasos, podemos observar como el robot va funcionando, aprendiendo.

Los fundamentos, explicación y metodología se encuentra explicada en la memoria.
