#include <yarp/os/Network.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Port.h>
#include <iostream >

using namespace yarp::os;

int main(int argc , char* argv[]) {
    // Declarar pero no usar argumentos
    YARP_UNUSED(argc);
    YARP_UNUSED(argv);

    Network yarp; // Inicializa YARP
    Bottle bot; // Botella para recibir mensajes
    Port input; // Puerto de entrada
    Network::connect("/custom_sender", "/receiver");
    // Abrir el puerto receptor
    input.open("/receiver");

    // Conexion directa entre puertos para este ejemplo
    Network::connect("/custom_sender", "/receiver");

    // Leer el mensaje recibido
    input.read(bot);
    std::cout << "Got message: " << bot.toString() << std::endl;
    
    // Cerrar el puerto receptor
    input.close();

    return 0;
}