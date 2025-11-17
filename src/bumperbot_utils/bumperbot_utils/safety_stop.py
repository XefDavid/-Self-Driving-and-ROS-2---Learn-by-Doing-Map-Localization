#!/usr/bin/env python3
import math
from enum import Enum
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from twist_mux_msgs.action import JoyTurbo

#Creamos una variable enum para manejar el estado del robot
class State(Enum):
    FREE = 0 #Cuando el robot no detecta obstaculos
    WARNING = 1 #Cuidado que hay un obstaculo cerca
    DANGER = 2 #Cuando el robot  esta a menos distancia de la marcada como minima 0.2

class SafetyStop(Node):

  #Creamos el contructor del nodo
    def __init__(self):
        super().__init__("safety_stop_node")

    #vamos a crear los parametros del nodo...
        self.declare_parameter("danger_distance", 0.2) #marca la distancia minima antes de parar..20cm.
        self.declare_parameter("warning_distance", 0.6)
        self.declare_parameter("scan_topic", "scan")        
        self.declare_parameter("safety_stop_topic","safety_stop")
    #Guardamos en las varibles los valores que recibimos de los paramentros y estandarizamos el tipo de valor
    #Para que los valores solo tengan dos decimales
        self.danger_distance = self.get_parameter("danger_distance").get_parameter_value().double_value 
        self.warning_distance = self.get_parameter("warning_distance").get_parameter_value().double_value 
    #Para estandarizar el valor de scan topic a string
        self.scan_topic = self.get_parameter("scan_topic").get_parameter_value().string_value
    #Para  estandarizar el valor de safety_stop_topic a string
        self.safety_stop_topic = self.get_parameter("safety_stop_topic").get_parameter_value().string_value   

    #Creamos una subscripcion al topic para guardar los mensajes
        self.laser_sub = self.create_subscription(LaserScan, self.scan_topic, self.laser_callback,10)
    #Creamos un publisher para decir si es true o false y si tiene que pararse o no el robot
        self.safety_stop_pub = self.create_publisher(Bool, self.safety_stop_topic, 10)

    # Creamos dod  objetos para manejar el warning y reducir o aumentar la speed
        self.decrease_speed_client = ActionClient(self,JoyTurbo, "joy_turbo_decrease")
        self.increase_speed_client = ActionClient(self, JoyTurbo, "joy_turbo_increase")

    # Comprobaciones para ver si el servido está activo , si no continua el node..
        while not self.decrease_speed_client.wait_for_server(timeout_sec = 1.0) and rclpy.ok():
            self.get_logger().warm("Action /joy_turbo_decrease not avalaible!! Waiting!!..")
        while not self.increase_speed_client.wait_for_server(timeout_sec = 1.0) and rclpy.ok():
                    self.get_logger().warm("Action /joy_turbo_increase not avalaible!! Waiting!!..")

    #Inicializamos el estadod del robot em free
        self.state = State.FREE
        self.prev_state = State.FREE


    def laser_callback(self, msg: LaserScan):
        #Manejamos el estado del robot poniendolo a valor free   
        self.state = State.FREE
        #Este loop revisa los valores para decidir si está dentro del rango Danger y parar el robot
        for range_value in msg.ranges:
            #si el valor es infinito o es menor a el valor marcado de danger_distance
            if not math.isinf(range_value) and range_value <= self.warning_distance:
                self.state = State.WARNING
                if range_value <= self.danger_distance:
                    self.state = State.DANGER
                    break

        if self.state != self.prev_state:
            is_safety_stop = Bool()
            #Si el valor es exactamente igual a DANGER => is_safety_stop.data = True...
            if self.state == State.WARNING:
                 is_safety_stop.data = False
                 self.decrease_speed_client.send_goal_async(JoyTurbo.Goal())
            elif self.state == State.DANGER:
                is_safety_stop.data = True
            elif self.state == State.FREE:
                is_safety_stop.data = False
                self.increase_speed_client.send_goal_async(JoyTurbo.Goal())

            self.prev_state = self.state
            self.safety_stop_pub.publish(is_safety_stop)

def main():
    rclpy.init()
    node = SafetyStop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
        

if __name__ == "__main__":
        main()




