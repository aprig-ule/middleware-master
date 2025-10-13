#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <iostream>
using namespace yarp::os;

int main() {
    Network yarp;

    // Crear productores y consumidor
    BufferedPort<Bottle> producer1;
    BufferedPort<Bottle> producer2;
    BufferedPort<Bottle> consumer;

    producer1.open("/example/producer1");
    producer2.open("/example/producer2");
    consumer.open("/example/consumer");

    // Conectar productores al consumidor
    Network::connect("/example/producer1", "/example/consumer");
    Network::connect("/example/producer2", "/example/consumer");

    int counter1 = 0;
    int counter2 = 0;

    while (true) {
        // Productor 1 envía mensaje cada 1s
        if (counter1 % 1000 == 0) {
            Bottle& msg1 = producer1.prepare();
            msg1.clear();
            msg1.addString("Producer1 says hello!");
            producer1.write();
        }

        // Productor 2 envía mensaje cada 2s
        if (counter2 % 2000 == 0) {
            Bottle& msg2 = producer2.prepare();
            msg2.clear();
            msg2.addString("Producer2 says hello!");
            producer2.write();
        }

        // Consumidor recibe y muestra mensajes
        Bottle* msg = consumer.read(false); // no bloqueante
        if (msg != nullptr) {
            std::cout << "Consumer received: " << msg->toString() << std::endl;
        }

        // Espera 1 ms y actualiza contadores
        yarp::os::Time::delay(0.001);
        counter1++;
        counter2++;
    }

    // Cerrar los puertos
    producer1.close();
    producer2.close();
    consumer.close();

    return 0;
}
