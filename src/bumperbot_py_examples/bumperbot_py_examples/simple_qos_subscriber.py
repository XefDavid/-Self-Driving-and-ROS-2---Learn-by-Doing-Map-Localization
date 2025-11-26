import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile,QoSDurabilityPolicy,QoSReliabilityPolicy

class SimpleQoSSubscriber(Node):

    def __init__(self):
        super().__init__("simple_qos_subscriber")
        self.qos_profile_sub = QoSProfile(depth=10)
        self.declare_parameter('reliability', 'system_default')
        self.declare_parameter('durability', 'system_default')

        reliability = self.get_parameter('reliability').get_parameter_value().string_value
        durability = self.get_parameter('durability').get_parameter_value().string_value

    # Lógica de confiabilidad
        if reliability == 'best_effort':
            self.qos_profile_sub.reliability = QoSReliabilityPolicy.BEST_EFFORT
            self.get_logger().info('[Reliability]:Best Effort')
        elif reliability == 'reliable':
            self.qos_profile_sub.reliability = QoSReliabilityPolicy.RELIABLE
            self.get_logger().info('[Reliability]: Reliable')
        elif reliability == 'system_default':
            self.qos_profile_sub.reliability = QoSReliabilityPolicy.SYSTEM_DEFAULT
            self.get_logger().info('[Reliability]: System_Default')
        else:
            self.get_logger().error(f'Selected Reliability QoS: % doesn\t exist! % reliability')
            return
    
    # Lógica de durabilidad
        if durability == 'volatile':
            self.qos_profile_sub.durability = QoSDurabilityPolicy.VOLATILE
            self.get_logger().info('[Durability]: Volatile')
        elif durability == 'transient_local':
            self.qos_profile_sub.durability  = QoSDurabilityPolicy.TRANSIENT_LOCAL
            self.get_logger().info('[Durability]: Transient_local')
        elif durability == 'system_default':
            self.qos_profile_sub.durability = QoSDurabilityPolicy.SYSTEM_DEFAULT
            self.get_logger().info('[Durability]: System_default')
        else:
            self.get_logger().error('Selected Durability Qos: % doesnt exist! % durability')
            return
    
   
        self.sub_ = self.create_subscription(String, "chatter", self.msgCallback, self.qos_profile_sub )
        self.sub_

    def msgCallback(self, msg):
        self.get_logger().info("I heard: %s" % msg.data)


def main(args=None):
    rclpy.init(args=args)

    simple_subscriber = SimpleQoSSubscriber()
    rclpy.spin(simple_subscriber)
    
    simple_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()