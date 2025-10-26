import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MultiCallbackNode(Node):
	def __init__(self):
		super().__init__('multi_callback_node')

		# Publicador
		self.publisher_ = self.create_publisher(String, 'example_topic', 10)
		self.timer = self.create_timer(1.0, self.publish_message)

		# Suscriptor
		self.subscription = self.create_subscription(
			String,
			'example_topic',
			self.listener_callback,
			10
		)
		self.subscription  # prevent unused variable warning

		self.get_logger().info('MultiCallbackNode is up and running.')

	def publish_message(self):
		msg = String()
		msg.data = 'Hello from MultiCallbackNode!'
		self.publisher_.publish(msg)
		self.get_logger().info(f'Published: "{msg.data}"')

	def listener_callback(self, msg):
		self.get_logger().info(f'Received: "{msg.data}"')


def main(args=None):
	rclpy.init(args=args)

	# Crear el nodo
	node = MultiCallbackNode()

	# Crear un MultiThreadedExecutor
	executor = rclpy.executors.MultiThreadedExecutor()
	executor.add_node(node)

	try:
		# Ejecutar el MultiThreadedExecutor
		executor.spin()
	except KeyboardInterrupt:
		pass
	finally:
		# Destruir el nodo y apagar ROS 2
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()