import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from bumperbot_msgs.action import Fibonacci
import time

class SimpleActionServer(Node):
    #Creamos el nodo 
    def __init__(self):
        super().__init__("simple_action_server")
        #Log para controlar el flujo
        self.get_logger().info("Starting action server")
        #Instanciamos el servidor con self(nodo), la interfaz Fibonacci(Goal,Result,Feedback), el nombre del topic(fibonacci) y la función que va a decir que hacer con el emnsaje que se recibe.
        self.action_server = ActionServer(self,Fibonacci,"fibonacci", self.goalCallback)

    def goalCallback(self, goal_handle):
        #Log que avisa de la llegada del objetivo 
        self.get_logger().info(f"Received goal request with order {goal_handle.request.order}")

        feedback_msg = Fibonacci.Feedback() #Creamos un mesaje tipo Fibonacci
        feedback_msg.partial_sequence = [0,1] #Iniciamos la primera secuencia con los dos primeros numeros

        for i in range(1,goal_handle.request.order):
            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i - 1])
            self.get_logger().info(f"Feedback: {feedback_msg.partial_sequence}")

            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)
        #Accion terminada con exito
        goal_handle.succeed()

        result = Fibonacci.Result() # we create a result message
        result.sequence = feedback_msg.partial_sequence
        return result
    
def main(arg=None):
    rclpy.init(args=None)
    simple_action_server = SimpleActionServer()
    rclpy.spin(simple_action_server)

if __name__ == "__main__":
    main()

