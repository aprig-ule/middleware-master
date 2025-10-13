# Ejercicio 1 YARP: Comandos Básicos de YARP en la línea de comandos


### 1. Verificar entorno de YARP
---
```yarp check```

Verifica si el servidor de nombres está activo.

---

```yarp conf <hostname> <port>```

Configura YARP para que use el servidor de nombres en la dirección y puerto indicados.  
Ejemplo:  
```yarp conf localhost 10000``` configura YARP para usar el servidor en tcp://localhost:10000

---

### 2. Iniciar y Verificar el Servidor de Nombres

---

```yarpserver```

Inicia el servidor de nombres de YARP.

---

```yarp where```

Muestra la dirección del servidor de nombres que está usando YARP.

---

### 3. Creación y manipulación de puertos

---

```yarp write /example/writer```

Crea un puerto de escritura llamado `/example/writer`.

---

```yarp read /example/reader```

Crea un puerto de lectura llamado `/example/reader`.

---

```yarp connect /example/writer /example/reader```

Conecta el puerto de escritura al puerto de lectura.

---

```yarp disconnect /example/writer /example/reader```

Desconecta los puertos previamente conectados.

---

```yarp name list```

Muestra la lista de puertos registrados en el servidor de nombres.

---

### 4. Pruebas y diagnóstico

---

```yarp connect /example/writer /example/reader```

Conecta los puertos recibidos por argumento.

```yarp ping /example/reader```

Verifica si el puerto `/example/reader` está disponible y responde.

---

```yarp read /example/reader```

Abre el puerto de lectura para recibir mensajes.

---

```yarp write /example/writer```

Abre el puerto de escritura para enviar mensajes.

---

### 5. Manipulación de puertos avanzada

---

```yarp write --anon```

Crea un puerto de escritura anónimo (sin nombre fijo).

---

```yarp name register /my/port```

Registra un puerto manualmente en el servidor de nombres.

---

```yarp name unregister /my/port```

Elimina el registro de un puerto en el servidor de nombres.

---

### 6. Limpieza del estado del Servidor de Nombres

---

```yarp clean```

Elimina registros 'huérfanos' o no usados en el servidor de nombres.

---

### 7. Comandos útiles para Redes Distribuídas

---

```yarp check --remote <hostname>:<port>```

Verifica el estado de un servidor de nombres remoto.

---

```yarp name reconfigure <port> <new-server>```

Cambia el servidor de nombres asociado a un puerto.


---
### 8. Configuración básica de puertos desde la línea de comandos.

**¿Qué sucede si intentas enviar un mensaje antes de conectar los puertos?**  

El mensaje no se entrega ya que no existe una conexión entre los puertos de lectura/escritura. El puerto de escritura mostrará un error o simplemente no enviará el mensaje.

**¿Cómo se desconectan los puertos desde la terminal?**  
Utilizando el comando:  
```yarp disconnect /reciver /producer```

**Prueba a crear un segundo puerto de lectura (/receiver2) y conéctalo al puerto de escritura. ¿Ambos lectores reciben el mensaje?**  
Si conectas el puerto de escritura a dos puertos de lectura (por ejemplo, `/receiver` y `/receiver2`), ambos puertos de lectura recibirán el mensaje enviado desde el puerto de escritura, siempre que estén correctamente conectados y abiertos.


# Ejercicio 2: Uso de puertos en YARP

### Creación de puertos desde nodos (programación en C++).
**¿Que sucede si los puertos no se conectan?**

El nodo productor envía los 50 mensajes, pero el nodo consumidor no los recibe.

**¿Cómo podrías modificar el nodo receptor para manejar multiples mensajes con diferentes formatos?**

HAbría que crear una función que despachase los mensajes según la etiqueta del Bottle. En función del tipo de dato se llevarían a cabo distintas operaciones (transformarlo a string, realizar distintos cálculos, etc).

### 1. Conexión Directa entre Puertos en YARP

**¿Cuáles son los pasos realizados?**
- Iniciar la red (Network yarp).

- Crear puertos de escritura y lectura (writerPort, readerPort) y abrirlos con nombres únicos para que puedan ser identificados.

- Conectarlos con writerPort.addOutput("/example/reader").

- Enviar mensaje con prepare() y write().

- Recibir mensaje con read().

- Cerrar puertos con close().

**¿Cuál es la diferencia entre Network::connect y Port::addOutput?**

Network::connect tiene un alcance más global (manejando las conexiones globales entre todos los puertos de un sistema), mientras que Port::addOutput se utiliza en un ámbito del propio puerto emisor con el receptor. Esto permite realizar conexiones locales de manera más sencilla.


**¿Cómo cambiarías los `cout` por mensajes de logging?**

En lugar de utilizar `std::cout` para mostrar mensajes en consola, se puede emplear una biblioteca de logging como `spdlog`. Esto permite configurar niveles de severidad, definir archivos de salida y añadir formato y timestamp automáticamente.

Ejemplo:

```cpp
#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>

// Inicializa el logger para escribir en logs.txt
auto logger = spdlog::basic_logger_mt("file_logger", "logs.txt");
spdlog::set_default_logger(logger);

// Reemplaza cout por log info
spdlog::info("Puerto abierto correctamente");
```

De esta forma, los mensajes se gestionan de manera más profesional y flexible que con `cout`.


# Ejercicio 3: BufferedPort en YARP

**Descripción del programa anterior**

El código muestra cómo establecer comunicación entre dos puertos en YARP usando BufferedPort para enviar y recibir mensajes tipo Bottle. Se crea un puerto de envío y otro de recepción, se conectan, se envía un mensaje y se recibe en el otro extremo. El uso de BufferedPort permite el intercambio de mensajes en "segundo plano" de manera que los hilos que producen o consumen no tienen que detenerse necesariamente a enviar/recibir cada mensaje, lo que facilita una comunicación asíncrona y confiable.

**Enumera los mas característico**

- Creación de puertos de tipo BufferedPort<Bottle>.
- Apertura de puertos con nombres únicos.
- Conexión directa entre puertos usando Network::connect.
- Preparación y envío de un mensaje tipo Bottle.
- Lectura y visualización del mensaje recibido.

# Ejercicio 3.1: BufferedPort en Comunicación Simultánea

Instrucciones:
1. Crea un programa en el que haya dos BufferedPort que actuen como productores (producer1 y producer2).
2. Ambos productores deben enviar mensajes periodicos al mismo puerto consumidor (consumer).
3. El puerto consumidor debe recibir y mostrar todos los mensajes enviados por los productores.

**Descripcion del proceso** 

Creación dos productores (productor1 y productor2) y un consumidor usando `BufferedPort` en YARP. 
Cada productor envía mensajes de manera periódica (productor1 cada 1s y productor2 cada 2s) al mismo puerto consumidor, que recibe y muestra todos los mensajes. 
El proceso demuestra cómo varios nodos pueden enviar datos simultáneamente a un único receptor, gestionando la concurrencia y evitando la pérdida de mensajes gracias al buffer interno de `BufferedPort`.

**Describe el codigo clave**

- Se crean dos puertos productores (`producer1`, `producer2`) y un puerto consumidor (`consumer`) de tipo `BufferedPort<Bottle>`.
- Los productores se conectan al consumidor usando `Network::connect`.
- Cada productor envía mensajes en intervalos distintos (1s y 2s) en bucle usando `prepare()` y `write()`.
- El consumidor lee los mensajes de forma no bloqueante con `read(false)` y los muestra por consola.

# Ejercicio 4: Tiempo real en YARP

Describe con tus palabras lo que consideres más relevante de los siguientes puntos:

- **Puerto del emisor (/sender):**  
    Puerto encargado de enviar mensajes de manera periódica. En el ejemplo, se utiliza un `BufferedPort<Bottle>` llamado `/sender` que genera y transmite datos a intervalos de 200ms.

- **Puerto del receptor (/receiver):**  
    Es el puerto que recibe los mensajes enviados por el sender. Se implementa como un `BufferedPort<Bottle>` llamado `/receiver`, encargado de leer y procesar los datos conforme llegan, permitiendo observar el comportamiento en tiempo real.

- **Frecuencia de envío:**  
    La frecuencia determina cada cuánto tiempo el emisor genera y envía mensajes al receptor. 

- **Ventajas y desventajas de setStrict():**  
    El método `setStrict()` en `BufferedPort` fuerza al puerto receptor a procesar todos los mensajes recibidos, sin descartar ninguno, haciendo que todos los mensajes sean procesados en orden de llegada, haciendo que el puerto receptor reciba los mensajes y los maneje como una cola FIFO (First In First Out).
    - *Ventajas:* Garantiza que no se pierda información, útil en aplicaciones donde cada mensaje es crítico (por ejemplo, adquisición de datos científicos).  
    - *Desventajas:* Si el receptor no puede procesar los mensajes a la misma velocidad que el emisor los envía, el buffer puede crecer y consumir mucha memoria, lo que puede afectar el rendimiento o provocar bloqueos si no se gestiona adecuadamente.


# Ejercicio 5: Multihilo en YARP


# Ejercicio 6: Relación entre YARP, ROS y ROS2


