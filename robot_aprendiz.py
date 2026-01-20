import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import random
import numpy as np

# En primer lugar definimos las posibles acciones del robot.
# 0: Avanzar
# 1: Girar Izquierda
# 2: Girar Derecha
ACCIONES = [0, 1, 2]

class RobotAprendiz(Node):
    def __init__(self):
        super().__init__('robot_q_learning')
        
        #Configurar la conexión con ROS 2:
        # Publicador para mover el robot
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Suscriptor para leer el láser
        self.sub = self.create_subscription(LaserScan, '/scan', self.sensor_callback, 10)
        
        #Incialización del cerebro (tabla Q)
        self.q_table = {}
        
        # Configuramos los parámetros de aprendizaje de inteligencia artificial
        self.alpha = 0.5       # Tasa de aprendizaje (0.5 = aprende rápido)
        self.gamma = 0.9       # Factor de descuento (importancia del futuro)
        self.epsilon = 0.9     # Curiosidad inicial (90% aleatorio al principio)
        self.epsilon_decay = 0.998 # Qué tan rápido deja de ser curioso
        self.min_epsilon = 0.05    # Curiosidad mínima (nunca deja de explorar del todo)
        
        # Variables de memoria
        self.last_state = "Desconocido"
        self.last_action = 0
        self.laser_data = None
        
        # Timer: El robot piensa cada 0,5 segundos,
        self.timer = self.create_timer(0.5, self.ciclo_aprendizaje)
        
        self.get_logger().info('¡El robot está listo para aprender!')

    def sensor_callback(self, msg):
        self.laser_data = msg

    def obtener_estado(self):
        #Traduce los 360 grados del láser a un estado simple de 3 bits
        if self.laser_data is None:
            return "Desconocido"
            
        ranges = self.laser_data.ranges
        # Definimos 3 zonas de visión (limitamos la distancia a 10m para evitar infinitos)
        # Frente: Un cono de 60 grados delante
        frente = min(min(ranges[0:30]), min(ranges[330:360]), 10.0)
        # Izquierda: Un cono de 60 grados a la izquierda
        izq = min(min(ranges[30:90]), 10.0)
        # Derecha: Un cono de 60 grados a la derecha
        der = min(min(ranges[270:330]), 10.0)
        
        # Discretizamos: 1 = LIBRE (Seguro), 0 = OBSTÁCULO (Peligro)
        # Umbral de seguridad: 0.5 metros
        umbral = 0.5
        s_frente = "1" if frente > umbral else "0"
        s_izq = "1" if izq > umbral else "0"
        s_der = "1" if der > umbral else "0"
        
        # El estado es la combinación, ej: "101" (Libre, Obstáculo, Libre)
        return s_izq + s_frente + s_der

    def elegir_accion(self, state):
        #Elige la mejor acción usando Epsilon-Greedy
        # Si el estado es nuevo, lo añadimos a la tabla con valores a 0
        if state not in self.q_table:
            self.q_table[state] = [0.0, 0.0, 0.0]

        # ¿Exploro (Curiosidad) o Exploto (Experiencia)?
        if random.uniform(0, 1) < self.epsilon:
            return random.choice(ACCIONES) # Acción aleatoria
        else:
            return np.argmax(self.q_table[state]) # La mejor acción aprendida

    def ciclo_aprendizaje(self):
        if self.laser_data is None:
            return

        # 1. OBSERVAR: Obtenemos el estado actual
        current_state = self.obtener_estado()
        
        # 2. CALCULAR RECOMPENSA (Basado en el resultado de la acción anterior)
        reward = 0
        distancia_minima = min(self.laser_data.ranges)
        
        if distancia_minima < 0.25: # CASO 1: CHOQUE
            reward = -100
            self.get_logger().error('¡Choque! Castigo: -100')
            
        elif self.last_action == 0: # CASO 2: AVANZAR
            reward = 10  
            self.get_logger().info('¡Bien hecho! Avanzando. Premio: +10')
            
        else: # CASO 3: GIRAR
            reward = -2  # Pequeño castigo por girar (para que no se quede quieto siempre)
            self.get_logger().warn('¡Continúe! Penalización leve: -2 ')
            
        # 3. APRENDER (Actualizar Tabla Q)
        if self.last_state in self.q_table:
            old_value = self.q_table[self.last_state][self.last_action]
            
            # Asegurar que el estado actual existe en la tabla
            if current_state not in self.q_table:
                 self.q_table[current_state] = [0.0, 0.0, 0.0]
                 
            # Mirar cuál es la mejor recompensa posible en el futuro
            next_max = np.max(self.q_table[current_state])
            
            # Ecuación de Bellman
            new_value = (1 - self.alpha) * old_value + self.alpha * (reward + self.gamma * next_max)
            self.q_table[self.last_state][self.last_action] = new_value

        # 4. DECIDIR NUEVA ACCIÓN
        action = self.elegir_accion(current_state)
        
        # 5. MOVER EL ROBOT
        msg = Twist()
        if action == 0:   # Avanzar
            msg.linear.x = 0.2
            msg.angular.z = 0.0
        elif action == 1: # Girar Izquierda
            msg.linear.x = 0.0
            msg.angular.z = 0.5
        elif action == 2: # Girar Derecha
            msg.linear.x = 0.0
            msg.angular.z = -0.5
            
        self.pub.publish(msg)

        # 6. GUARDAR PARA LA SIGUIENTE VUELTA
        self.last_state = current_state
        self.last_action = action
        
        # Reducir la curiosidad (Epsilon)
        if self.epsilon > self.min_epsilon:
            self.epsilon *= self.epsilon_decay

def main(args=None):
    rclpy.init(args=args)
    node = RobotAprendiz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Al cerrar, parar el robot
        stop_msg = Twist()
        node.pub.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
