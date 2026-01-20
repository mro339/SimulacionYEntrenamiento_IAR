import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
import random
import numpy as np
import time
import math

#Posibles acciones:
# 0: Avanzar, 1: Girar Izquierda, 2: Girar Derecha
ACCIONES = [0, 1, 2]

class RobotLaboratorioEpsilon(Node):
    def __init__(self):
        super().__init__('robot_iterativo_epsilon')

        #Comunicación con ROS
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(LaserScan, '/scan', self.sensor_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.reset_client = self.create_client(Empty, '/reset_simulation')

        # Configuración de la pruebas iterativa, epsilon.
        # Lista de valores iniciales de Epsilon a probar
        self.lista_epsilons = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9]
        self.indice_actual = 0

        self.duracion_ronda = 5 * 60  # 5 minutos por ronda

        # Inicializamos variables
        self.q_table = {}
        self.laser_data = None
        self.robot_flipped = False

        # Iniciamos la primera ronda
        self.iniciar_ronda()

        self.timer = self.create_timer(0.5, self.ciclo_aprendizaje)

    def iniciar_ronda(self):
        #Preparación para probar cada valor de epsilón
        # 1. Actualizamos el valor correspondiente del epsilon.
        self.epsilon_inicial = self.lista_epsilons[self.indice_actual]
        self.epsilon = self.epsilon_inicial # Asignamos el valor actual

        # 2. Fijamos otros hiperparámetros
        self.alpha = 0.5
        self.gamma = 0.9
        self.epsilon_decay = 0.998 # Velocidad a la que pierde la curiosidad
        self.min_epsilon = 0.05

        # 3. Reiniciamos la memoria, para empezar de nuevo, y obtener los resultados correspondientes al nuevo epsilon
        self.q_table = {}
        self.last_state = "Desconocido"
        self.last_action = 0
        self.start_time = time.time()
        self.total_reward = 0
        self.total_crashes = 0
        self.total_resets = 0
        self.steps =0
        self.total_forward = 0
        self.total_turns = 0

        # 4. Riniciamos simulación
        self.reset_simulation()
        #Para una mejor intrepretación y entendimiento hace que nos muestre en el terminal la información.
        self.get_logger().info('='*40)
        self.get_logger().info(f'🧪 INICIANDO RONDA {self.indice_actual + 1}/{len(self.lista_epsilons)}')
        self.get_logger().info(f'  Probando EPSILON INICIAL = {self.epsilon} (Curiosidad)')
        self.get_logger().info('='*40)

    def sensor_callback(self, msg):
        self.laser_data = msg

    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        roll, pitch, yaw = self.euler_from_quaternion(orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
        if abs(roll) > 1.0 or abs(pitch) > 1.0:
            self.robot_flipped = True

    def euler_from_quaternion(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        return roll_x, pitch_y, yaw_z

    def reset_simulation(self):
        req = Empty.Request()
        while not self.reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Esperando reset...')
        self.reset_client.call_async(req)
        time.sleep(1.0)
        self.robot_flipped = False

    def obtener_estado(self):
        if self.laser_data is None:
            return "Desconocido"
        ranges = self.laser_data.ranges
        frente = min(min(ranges[0:30]), min(ranges[330:360]), 10.0)
        izq = min(min(ranges[30:90]), 10.0)
        der = min(min(ranges[270:330]), 10.0)
        umbral = 0.5
        return ("1" if izq > umbral else "0") + ("1" if frente > umbral else "0") + ("1" if der > umbral else "0")

    def elegir_accion(self, state):
        if state not in self.q_table:
            self.q_table[state] = [0.0, 0.0, 0.0]

        # Actuación del epsilon, con el que estamos haciendo el estudio
        if random.uniform(0, 1) < self.epsilon:
            return random.choice(ACCIONES) # Exploración
        else:
            return np.argmax(self.q_table[state]) # Explotación

    def ciclo_aprendizaje(self):
        # 1. Reiniciamos una vez, que el robot vuelque
        if self.robot_flipped:
            self.total_resets += 1
            self.total_crashes += 1
            self.total_reward -= 200
            self.get_logger().error('🚨 ROBOT VOLCADO -> REINICIO')
            self.reset_simulation() #Simplemente reiniciamos, al contrario que cuando empieza una nueva ronda con un nuevo valor de epsilon, que en ese caso, a parte de reiniciar, se borra la memoria 
            return

        # 2. Comprobamos el tiempo
        elapsed_time = time.time() - self.start_time
        if elapsed_time > self.duracion_ronda:
            self.finalizar_ronda()
            return

        if self.laser_data is None:
            return

        # Aprendizaje
        current_state = self.obtener_estado()
        reward = 0
        distancia_minima = min(self.laser_data.ranges)

        # Recompensas
        if distancia_minima < 0.25:
            reward = -100
            self.total_crashes += 1
            self.get_logger().error(f'CHOQUE (-100) ({self.total_crashes})')

        elif self.last_action == 0:
            reward = 10
            self.get_logger().info('Avanzando (+10)')

        else:
            reward = -2
            self.get_logger().info('Girando (-2))')

        self.total_reward += reward
        self.steps += 1

        if self.last_state in self.q_table:
            old_value = self.q_table[self.last_state][self.last_action]
            if current_state not in self.q_table:
                 self.q_table[current_state] = [0.0, 0.0, 0.0]
            next_max = np.max(self.q_table[current_state])

            # Ecuación de Bellman (Alpha y Gamma fijos)
            new_value = (1 - self.alpha) * old_value + self.alpha * (reward + self.gamma * next_max)
            self.q_table[self.last_state][self.last_action] = new_value

        action = self.elegir_accion(current_state)

        if action == 0:
            self.total_forward += 1
        else:
            self.total_turns += 1

        msg = Twist()
        if action == 0:
            msg.linear.x = 0.2
            msg.angular.z = 0.0
        elif action == 1:
            msg.linear.x = 0.0
            msg.angular.z = 0.5
        elif action == 2:
            msg.linear.x = 0.0
            msg.angular.z = -0.5
        self.pub.publish(msg)

        self.last_state = current_state
        self.last_action = action

        # Decaimiento de Epsilon
        if self.epsilon > self.min_epsilon:
            self.epsilon *= self.epsilon_decay

    def finalizar_ronda(self):
        # Informe final con toda la información que ha ido recopilando
        print("\n" + "-"*50)
        print(f"RESULTADOS RONDA {self.indice_actual + 1} (Epsilon Inicial = {self.epsilon_inicial})")
        print("-"*50)
        print(f"Puntuación: {self.total_reward}")
        print(f"Accidentes: {self.total_crashes} (Vuelcos: {self.total_resets})")
        print(f"Estilo: Avanza {self.total_forward} veces | Gira {self.total_turns} veces")
        print("-"*50 + "\n")

        self.indice_actual +="-"
        if self.indice_actual < len(self.lista_epsilons):
            self.iniciar_ronda()
        else:
            print("¡EXPERIMENTO DE EPSILON COMPLETADO! CERRANDO...")
            stop_msg = Twist()
            self.pub.publish(stop_msg)
            self.destroy_node()
            rclpy.shutdown()
            exit()

def main(args=None):
    rclpy.init(args=args)
    node = RobotLaboratorioEpsilon()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit:
        pass

if __name__ == '__main__':
    main()

