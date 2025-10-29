#include <yarp/os/all.h>
#include <iostream>
#include <string>
#include <thread>

using namespace yarp::os;
using namespace std;

class SenderThread : public Thread {
private:
    BufferedPort<Bottle> portOut; // Port to send messages
    int counter;

public:
    SenderThread() : counter(0) {}

    bool threadInit() override {
        // Open the output port
        return portOut.open("/sender/out");
    }

    void run() override {
        while (!isStopping()) {
            Bottle& msg = portOut.prepare();
            msg.clear();
            msg.addString("Message #" + std::to_string(counter++));
            portOut.write(); // Send the message
            cout << "[Sender] Message sent" << endl;
            yarp::os::Time::delay(2.0); 
        }
    }

    void threadRelease() override {
        cout << "[Sender] Closing port..." << endl;
        portOut.close();
    }
};


class ReceiverThread : public Thread {
private:
    BufferedPort<Bottle> portIn;

public:
    bool threadInit() override {
        // Open the input port
        return portIn.open("/receiver/in");
    }

    void run() override {
        while (!isStopping()) {
            Bottle* msg = portIn.read(false);
            if (msg != nullptr) {
                cout << "[Receiver] Received: " << msg->toString() << endl;
            }
            yarp::os::Time::delay(2);
        }
    }

    void threadRelease() override {
        cout << "[Receiver] Closing port..." << endl;
        portIn.close();
    }
};


int main(int argc, char* argv[]) {
    Network yarp; // Initialize the YARP network
    if (!yarp.checkNetwork()) {
        cerr << "Error: YARP is not available." << endl;
        return -1;
    }

    // Create threads
    SenderThread sender;
    ReceiverThread receiver;

    // Connect the ports
    sender.start();
    receiver.start();
    yarp.connect("/sender/out", "/receiver/in");

    cout << "Press ENTER to stop the threads..." << endl;
    cin.get();

    // Stop the threads
    sender.stop();
    receiver.stop();

    cout << "FInished successfully." << endl;
    return 0;
}
