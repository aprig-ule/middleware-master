import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

class QoSPublisher(Node):
    def __init__(self):
        super().__init__('qos_publisher')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=10)
        self.publisher_ = self.create_publisher(String, 'qos_topic', qos_profile)
        self.timer = self.create_timer(2.0, self.publish_message)
        self.count = 0

    def publish_message(self):
        msg = String()
        msg.data = f'QoS mensaje #{self.count}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Enviado: {msg.data}')
        self.count += 1

class QoSSubscriber(Node):
    def __init__(self):
        super().__init__('qos_subscriber')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10)
        self.subscriber = self.create_subscription(String, 'qos_topic', self.callback, qos_profile)

    def callback(self, msg):
        self.get_logger().info(f'Recibido: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    pub = QoSPublisher()
    sub = QoSSubscriber()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(pub)
    executor.add_node(sub)
    executor.spin()
    pub.destroy_node()
    sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
