import rclpy
from rclpy.node import Node
#esta libreria se usa para pedir acciones a un servidor
from rclpy.action import ActionClient 
#Importamos la interfazde acciones que define la estructura de los mensajes Goal, Result y fibonacci.
from bumperbot_msgs.action import Fibonacci 

class SimpleActionClient(Node):

    def __init__(self):
        super().__init__("simple_action_client")
        # Creamos el objeto action_client,segun libreria ros2. 
        # fibonacci = action_Server donde enviamos la Goal;
        # Fibonacci = interface;
        self.action_client = ActionClient(self, Fibonacci, "fibonacci")
        self.goal = Fibonacci.Goal()#Crea un nuevo mensaje de  objetivo Goal para enviar al servidor.
        self.goal.order = 10 #Calcula Fibonacci hasta el orden 10.

        # Detenemos la ejecución del script hasta que el servidor action Fibonacci este listo y activo para recibir notificaciones. 
        self.action_client.wait_for_server()

        # Enviamos la goal de forma asyncrona, asi no bloqueamos el nodo.
        # Si hay respuesta del servidor debe de usar la funcion feedbackCallback
        self.future = self.action_client.send_goal_async( self.goal,feedback_callback = self.feedbackCallback )

        # Si el servidor responde si se acepto o no la goal , ejecuta la función.
        self.future.add_done_callback(self.responseCallback)
    
    def responseCallback(self,future):
        # Obtiene la respuesta del servidor (acepta o no la Goal??)
        goal_handle =  future.result()

        # Si la Goal no está aceptada...
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected:(')")
            return
        
        self.get_logger().info("Goal accepted:(')") # Goal accepted!!

        self.future = goal_handle.get_result_async() #Solicita asincronamente el resultado de la action.
        # Cuando el servidor haya terminado la tarea (calculo fibonacci), ejecuta la función.
        self.future.add_done_callback(self.resultCallback)
    
    def resultCallback(self,future):
        # Obtenemos el resultado del calculo y lo guardamos en result
        result = future.result().result
        self.get_logger().info('Result:{0}'.format(result.sequence))
        rclpy.shutdown() # Una vez obtenido el resultado se cierra el nodo.

    def feedbackCallback (self, feedback_msg):
        # Extrae el mensaje del progreso recibido
        feedback = feedback_msg.feedback
        #Muestra la secuencia parcial de fibonacci que se ha calculado hasta el momento.
        self.get_logger().info('Received feedback: {0}'.format(feedback.partial_sequence))

def main(args = None):
    rclpy.init(args = args)
    action_client = SimpleActionClient()
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
