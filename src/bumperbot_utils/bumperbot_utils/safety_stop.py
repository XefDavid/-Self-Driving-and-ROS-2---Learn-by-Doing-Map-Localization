#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class SafetyStop(Node):

  #Creamos el contructor del nodo
    def __init__(self):
        super().__init__("safety_stop_node")

    #vamos a crear los parametros del nodo...
        self.declare_parameter("danger_distance", 0.2) #marca la distancia minima antes de parar..20cm.
        self.declare_parameter("scan_topic", "scan")        
        self.declare_parameter("safety_stop_topic","safety_stop")
    #Guardamos en las varibles los valores que recibimos de los paramentros y estandarizamos el tipo de valor
    #Para que los valores solo tengan dos decimales
        self.danger_distance = self.get_parameter("danger_distance").get_parameter_value().double_value 
    #Para estandarizar el valor de scan topic a string
        self.scan_topic = self.get_parameter("scan_topic").get_parameter_value().string_value
    #Para  estandarizar el valor de safety_stop_topic a string
        self.safety_stop_topic = self.get_parameter("safety_stop_topic").get_parameter_value().string_value   

