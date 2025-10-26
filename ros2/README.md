# Exercise 1 ROS2: Publishers y subscribers


### 1.1 Minimal Publisher
Se implementa un nodo que publica mensajes de texto en el tópico `chatter` cada 0.5 segundos:

```python
class MinimalPublisher(Node):
	def __init__(self):
		super().__init__('minimal_publisher')
		self.publisher_ = self.create_publisher(String, 'chatter', 10)
		timer_period = 0.5  # seconds
		self.timer = self.create_timer(timer_period, self.timer_callback)
		self.i = 0

	def timer_callback(self):
		msg = String()
		msg.data = 'Hola ROS 2: %d' % self.i
		self.publisher_.publish(msg)
		self.get_logger().info('Publicando: "%s"' % msg.data)
		self.i += 1
```

### 1.2 Minimal Subscriber
Se implementa un nodo que se suscribe al tópico `chatter` y muestra por consola los mensajes recibidos:

```python
class MinimalSubscriber(Node):
	def __init__(self):
		super().__init__('minimal_subscriber')
		self.subscription = self.create_subscription(
			String,
			'chatter',
			self.listener_callback,
			10)

	def listener_callback(self, msg):
		self.get_logger().info('Recibido: "%s"' % msg.data)
```

### 1.3 Publisher and subscriber custom messages
En este ejercicio se modificaría el tipo de mensaje publicado y recibido por los nodos para utilizar un mensaje personalizado en vez de `std_msgs.msg.String`. El código sería similar, pero cambiando la importación y el tipo de mensaje:

```python
 from my_custom_msgs.msg import CustomMsg
 ...
 self.publisher_ = self.create_publisher(CustomMsg, 'custom_topic', 10)
 ...
 self.subscription = self.create_subscription(
     CustomMsg,
     'custom_topic',
     self.listener_callback,
     10)
```

### 1.4 Publisher with timer
El publisher ya hace uso de un temporizador para publicar mensajes periódicamente, como se muestra en el fragmento anterior de `MinimalPublisher`:

```python
timer_period = 0.5  # seconds
self.timer = self.create_timer(timer_period, self.timer_callback)
```

### 1.5 Subscriber with custom callback
El subscriber define un callback personalizado que procesa cada mensaje recibido:

```python
def listener_callback(self, msg):
	self.get_logger().info('Recibido: "%s"' % msg.data)
```

---

Se ha creado un paquete "py_pubsub" siguiendo la documentación oficial de ROS2:

```bash
ros2 pkg create --build-type ament_python py_pubsub
```

Dentro de este paquete se encuentran los scripts para publisher y subscriber.

### Instrucciones para ejecutar el paquete py_pubsub

1. Preparar entorno ROS 2
- Cargar el setup de ROS2 Jazzy
```bash
source /opt/ros/Jazzy/setup.bash
```

2. Construir el workspace
- Desde la raíz del workspace que contiene la carpeta `src` (donde está `py_pubsub`):

```bash
colcon build --packages-select py_pubsub --symlink-install
```
- Tras la compilación, cargar el setup del workspace:

```bash
source install/setup.bash
```

3. Ejecutar nodos

- Se ha de añadir los entrypoints al setup.py
```bash
python3 src/py_pubsub/py_pubsub/minimal_publisher.py
python3 src/py_pubsub/py_pubsub/minimal_subscriber.py
```

- En terminales distintas (cada una requiere lanzar `source install/setup.bash`):

    - Ejecutar el publisher:
    ```bash
    ros2 run py_pubsub minimal_publisher
    ```

    - Ejecutar el subscriber:
    ```bash
    ros2 run py_pubsub minimal_subscriber
    ```



4. Se verifican los tópicos y se testea en `chatter`

- Listar tópicos:
```bash
ros2 topic list
```

- Mostrar mensajes del tópico `chatter`:
```bash
ros2 topic echo /chatter
```



### 1.6 Pub/Sub with QoS
En este ejercicio se implementa un único nodo que actúa como publicador y suscriptor, cada uno con un perfil de Calidad de Servicio (QoS) diferente:

- **Publicador**: Publica mensajes en el tópico `durable_topic` usando un perfil QoS confiable y duradero (`RELIABLE`, `TRANSIENT_LOCAL`), que asegura que los mensajes se almacenen y puedan ser recibidos por suscriptores que se conecten más tarde.

- **Suscriptor**: Se suscribe al tópico `volatile_topic` usando un perfil QoS de mejor esfuerzo y volátil (`BEST_EFFORT`, `VOLATILE`), lo que prioriza la inmediatez sobre la fiabilidad y no almacena mensajes.

```python
class PubSubNode(Node):
	def __init__(self):
		super().__init__('pub_sub_node')

		pub_qos = QoSProfile(
			reliability=ReliabilityPolicy.RELIABLE,
			durability=DurabilityPolicy.TRANSIENT_LOCAL,
			depth=10
		)
		self.publisher_ = self.create_publisher(String, 'durable_topic', pub_qos)

		sub_qos = QoSProfile(
			reliability=ReliabilityPolicy.BEST_EFFORT,
			durability=DurabilityPolicy.VOLATILE,
			depth=10
		)
		self.subscription = self.create_subscription(
			String,
			'volatile_topic',
			self.listener_callback,
			sub_qos
		)

		self.timer = self.create_timer(2.0, self.publish_message)

		self.get_logger().info('PubSubNode está funcionando.')

	def publish_message(self):
		msg = String()
		msg.data = 'Mensaje desde durable_topic'
		self.publisher_.publish(msg)
		self.get_logger().info(f'Publicado en durable_topic: "{msg.data}"')

	def listener_callback(self, msg):
		self.get_logger().info(f'Recibido en volatile_topic: "{msg.data}"')
```

El método `publish_message` publica periódicamente en el tópico `durable_topic`, mientras que `listener_callback` procesa los mensajes recibidos en `volatile_topic`.

```python
def main(args=None):
	rclpy.init(args=args)
	node = PubSubNode()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()
```

### 1.8 Publisher/Subscriber with queues


### 1.12 Multithread management with executors


# Exercise 2 ROS2: Subscription to topics via CLI

### 2.1 Basic configuration
ros2 topic echo /example_topic

### 2.2 Advanced configurations of QoS

### 2.3 QoS param mixing

### 2.4 Additional QoS config

### 2.5 Topic monitoring



# Exercise 3 ROS2: Concurrence management and callbacks

### 3.1 Publisher/Subscriber with callback groups

### 3.2 Multiple nodes in a single process

### 3.3 Node with publisher and subscriber


# Exercise 4 ROS2: Service that counts vowels

