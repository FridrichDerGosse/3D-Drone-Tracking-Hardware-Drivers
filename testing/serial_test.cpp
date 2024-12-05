#include <string>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include "drivers/CppLinuxSerial/SerialPort.hpp"

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()


using json = nlohmann::json;
using namespace mn::CppLinuxSerial;


int main()
{
    SerialPort serialPort("/dev/ttyUSB0", BaudRate::B_9600, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE);
    serialPort.SetTimeout(100); // Block for up to 100ms to receive data
	serialPort.Open();

    std::cout << "started" << std::endl;

    // request laser range
    json request = {
        {"type", 2}
    };

    // send request to nano
    serialPort.Write(request.dump());

    std::string readData;
    for (;;)
    {
        // wait for reply
        if (serialPort.Available())
        {
            serialPort.Read(readData);

            // convert data to json
            json reply = json::parse(readData);
            std::cout << "laser request data:" << reply << std::endl;

            return 0;
        }

        // wait for 10ms
        usleep(10000);
    }
}
