import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

class Publisher(Node):
    def __init__(self):
        super().__init__('multi_pub')
        self.publisher_ = self.create_publisher(String, 'multi_topic', 10)
        self.timer = self.create_timer(0.5, self.publish_message)
        self.i = 0

    def publish_message(self):
        msg = String()
        msg.data = f'Mensaje #{self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publicado: {msg.data}')
        self.i += 1

class Subscriber(Node):
    def __init__(self):
        super().__init__('multi_sub')
        self.subscription = self.create_subscription(String, 'multi_topic', self.callback, 10)

    def callback(self, msg):
        self.get_logger().info(f'Recibido: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    publisher = Publisher()
    subscriber = Subscriber()
    executor = MultiThreadedExecutor()
    executor.add_node(publisher)
    executor.add_node(subscriber)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        publisher.destroy_node()
        subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
