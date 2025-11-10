#!/usr/bin/env_python

from math import sin, cos, atan2, sqrt,fabs, pi
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import random
import time

def normalize (z):
    return atan2(sin(z),cos(z)) #normaliza el angulo al rango pi y menos pi , asegunrando consistencia.

#esta funcion calcula la diferencia angular mas corta entre dos angulos con direccion , tb normalizada
def angle_diff(a,b):
    #normalizamos el angulo
    a = normalize(a)
    b = normalize(b)
    #calcula diferencia simple
    d1 = a - b
    d2 = 2 * pi -fabs(d1) # calcula la diferencia simple, multiplicando 2 por pi y restandole el arcotangente de d1
    if d1 > 0:
        d2 *= -1.0
    if fabs(d1) < fabs(d2):
        return d1
    else:
        return d2
    
class OdometryMotionModel(Node):
    def __init__(self):
        super().__init__('odometry_motion_model')
        # Vamos a inicializar unas varibles a cero para almacenas las ultimas posiciones conocidas de Odometria
        self.last_odom_x = 0.0
        self.last_odom_y = 0.0
        self.last_odom_theta = 0.0
        self.is_first_odom = True #comprobacion para ver que es el primer mensaja que recibe

        # Vamos a declarar parametros , para añadir ruido 
        self.declare_parameter('alpha1', 0.1)
        self.declare_parameter('alpha2',0.1)
        self.declare_parameter('alpha3',0.1)
        self.declare_parameter('alpha4',0.1)
        self.declare_parameter('nr_samples',300) #numero de muestras que vamos a coger

        # Hacemos que los valores solo tengan dos decimales
        self.alpha1 = self.declare_parameter('alpha1').get_parameter_value().double_value
        self.alpha2 = self.declare_parameter('alpha2').get_parameter_value().double_value
        self.alpha3 = self.declare_parameter('alpha3').get_parameter_value().double_value
        self.alpha4 = self.declare_parameter('alpha4').get_parameter_value().double_value
        self.nr_samples = self.declare_parameter('nr_samples').get_parameter_value().double_value

        #Esta condicional sirve para crear un array de poses de ros2 , si ya hay mensajes en nr_samples
        if self.nr_samples >=0:
            self.samples = PoseArray()
            self.samples.poses = [Pose() for _ in range(self.nr_samples)]
        else :
            self.get_logger().fatal(f"Invalid number of sample request {self.nr_samples} exit")
            return
        
        #Creamos tanto el subscriber como el publisher
        self.odom_sub = self.create_subscription(Odometry,'bumperbot_controller/odom',self.odom_callback,10)
        self.pose_Array_pub = self.create_publisher(PoseArray,'Odometry_motion_model/samples',10)

    def odom_callback(self,odom):
        #Convierte los quaternion a grados euler
        q =  [
            odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,odom.pose.pose.orientation.w
        ]
        roll,pitch,yaw = euler_from_quaternion(q)

        if self.is_first_odom:
            #establece el frame de referencia al PoseArray a partir del mensaje de odometria
            self.samples.header.frame_id = odom.header.frame_id 
            #Establece los datos de la ultima posicion de odometria conocida
            self.last_odom_x = odom.pose.pose.position.x
            self.last_odom_y = odom.pose.pose.position.y
            self.last_odom_theta = yaw
            self.is_first_odom = False #Esto desactiva el flag, cierra la función y espera nuevos mensajes
            return
        
        #Calculamos el incremento para saber la variacion de la posicion
        odom_x_increment = odom.pose.pose.position.x - self.last_odom_x
        odom_y_increment = odom.pose.pose_position.y - self.last_odom_y
        odom_theta_increment = angle_diff(yaw,self.last_odom_theta)

        if sqrt(pow(odom_y_increment,2)+pow(odom_x_increment,2))<0.01:
            delta_rot1 = 0.0
        else:
            delta_rot1 = angle_diff(atan2(odom_y_increment,odom_x_increment),yaw)

        delta_trans = sqrt(pow(odom_x_increment,2)+ pow(odom_y_increment,2),yaw)
        delta_rot2 = angle_diff(odom_theta_increment,delta_rot1)
        
        #Genera variación de ruido
        rot1_variance = self.alpha1 * delta_rot1 + self.alpha2 * delta_trans
        trans_variance = self.alpha3 * delta_trans + self.alpha4 * (delta_rot1 + delta_rot2)
        rot2_variance = self.alpha1 * delta_rot2 + self.alpha2 * delta_trans

        #genera enteros de timpo, no decimales
        random.seed(int(time.time()))

        #recorremos el array que esta lleno de poses.....hechas con PoseArray
        for sample in self.poses:
            rot1_noise = random.gauss (0.0, rot1_variance)
            rot2_noise = random.gauss (0.0, rot2_variance)
            trans_noise = random.gauss(0.0, trans_variance)
            #aplicamos el ruido como un movimiento calculado el ruido se aplica como una diferencia angular
            delta_rot1_draw = angle_diff(delta_rot1,rot1_noise)
            delta_rot2_draw = angle_diff(delta_rot2, rot2_noise)
            delta_trans_draw = delta_trans - trans_noise
            #extraemos la orientacion de la muestra actual y la convertimos en yaw
            sample_q = [sample.orientation.x, sample.orientation.y,
                        sample.orientation.z,sample.orientation.w]
            sample_roll, sample_pitch,sample_yaw = euler_from_quaternion(sample_q)
            #Actualiza la pose, la translacion,proyectada en el eje, usando orientacion de la muestra + el giro inicial
            sample.position.x += delta_trans_draw + cos(sample_yaw + delta_rot1_draw)  
            sample.position.y += delta_trans_draw + sin(sample_yaw + delta_rot1_draw)
            #
            q= quaternion_from_euler(0.0,0.0,sample_yaw + delta_rot1_draw + delta_rot2_draw )
            sample.orientation.x,sample.orientation.y,sample.orientation.z,sample.orientation.w = q
        #Almacena la posicion y orientacion actualesv como la actual posicion conocida
        self.last_odom_x = odom.pose.pose.position.x
        self.last_odom_y = odom.pose.pose.position.y
        self.last_odom_theta = yaw

        self.pose_Array_pub.publish(self.samples)

def main():
    rclpy.init
    node = OdometryMotionModel()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__== '__main__':
    main()


