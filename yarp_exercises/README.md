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

El propósito de Network::connect tiene un alcance más global (manejando las conexiones globales entre todos los puertos de un sistema), mientras que Port::addOutput se utiliza en un ámbito del propio puerto emisor con el receptor. Esto permite realizar conexiones locales de manera más sencilla.


**¿Como cambiarías los "cout" por mensajes de logging?**


# Ejercicio 3: BufferedPort en YARP


# Ejercicio 4: Tiempo real en YARP


# Ejercicio 5: Multihilo en YARP


# Ejercicio 6: Relación entre YARP, ROS y ROS2


