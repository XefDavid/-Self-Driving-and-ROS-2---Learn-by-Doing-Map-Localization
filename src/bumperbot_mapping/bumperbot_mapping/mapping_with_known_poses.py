#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
# Esta importacion sirve para usar mensajes que representan una cuadricula y sus métodos
from nav_msgs.msg import OccupancyGrid, MapMetaData
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener,LookupException

class Pose:
    # Esta clase representa una posición en las coordenadas de celda con x e y enteros.
    # Se usa internamente para recorrer celdas.
    def __init__(self, px = 0, py = 0):
        self.x = px
        self.y = py

    # Esta función convierte coordenadas en metros(px,py) a indices de celda usando el origen y el tamaño d ela celda del mapa (resolucion)    
def coordinatesToPose(px, py, map_info: MapMetaData):
    pose = Pose()
    pose.x = round((px - map_info.origin.position.x)/ map_info.resolution)
    pose.y = round((py -map_info.origin.position.y)/ map_info.resolution)
    return pose

def poseOnMap(pose: Pose, map_info: MapMetaData):
    return pose.x < map_info.width and pose.x>=0 and pose.y < map_info.height and pose.y >= 0 

def poseToCell(pose:Pose, map_info: MapMetaData):
    return map_info.width  * pose.x + pose.y

class MappingWithKnownPoses(Node):
    def __init__(self, name):
        super().__init__(name)

    # Marcamos las medidas de ña grilla en metros...y la resolución en cm(tamaño de la celda).
        self.declare_parameter("width", 50.0)
        self.declare_parameter("height", 50.0)
        self.declare_parameter("resolution", 0.1)

        width = self.get_parameter("width").value
        height = self.get_parameter("height").value
        resolution = self.get_parameter("resolution").value

        self.map_ = OccupancyGrid() # Creamos un mensaje con la nomenclatura de nav_msgs/OccupancyGrid 
        self.map_.info.resolution = resolution
        self.map_.info.width = round(width / resolution) # Da el numero de celdas
        self.map_.info.height = round(height / resolution) # Da el numero de celdas de altura
        self.map_.info.origin.position.x = float(-round(width / 2.0) ) # Busca el punto central del mapa a lo ancho
        self.map_.info.origin.position.y = float(-round(height / 2.0)) # Busca el punto central del mapa a lo alto
        self.map_.header.frame_id = "odom"
        self.map_.data =[-1]*(self.map_.info.width * self.map_.info.height) # Esto es area del mapa

        # Creamos un publicador del topic map con OccupancyGrid
        self.map_pub = self.create_publisher(OccupancyGrid, "map", 1) 
        # Creamos una subscripcion al topic LaserScan
        self.map_sub = self.create_subscription(LaserScan, "scan", self.scan_callback, 10) 
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
    
    def scan_callback(self, scan: LaserScan):
        # Con esta función lo que pretendemos es almacenar la transformacion entre el scan que viene del topic scan y 
        # el frame id => odom 
        try:
            t = self.tf_buffer.lookup_transform(self.map_.header.frame_id,scan.header.frame_id, rclpy.time.Time())
        except LookupException:
            self.get_logger().error("Unable to transform between /odom and /base_footprint")
            return
        # Convertimos en coordenadadas de transformacion para alamacenar la posicion del robot en el ocupancygrid
        robot_p = coordinatesToPose(t.transform.translation.x, t.transform.translation.y, self.map_.info)
        if not poseOnMap (robot_p, self.map_.info):
            self.get_logger().error("The robot is out of the map")
            return
        robot_cell = poseToCell(robot_p, self.map_.info)
        self.map_.data[robot_cell] = 100

    def timer_callback(self):
        self.map_.header.stamp = self.get_clock().now().to_msg()
        self.map_pub.publish(self.map_)

def main():
    rclpy.init()
    node = MappingWithKnownPoses("mapping_with_known_poses")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()