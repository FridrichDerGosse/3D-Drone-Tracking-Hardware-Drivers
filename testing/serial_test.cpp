#include <string>
#include <fstream>
#include <unistd.h>
#include <iostream>
#include "drivers/serial.hpp"


int main() {
    serial::SimpleSerial nano("/dev/ttyUSB0");
    nano.begin(9600);

    std::cout << "Setup device " << std::endl;

    // wait two seconds
    usleep(2000000);

    // Write to the serial device
    const char* message = "{\"type\": 2}\0";
    int bytes_written = nano.write(message);

    std::cout << "Sent " << bytes_written << " bytes to the serial device." << std::endl;

    // Read from the serial device
    while (!nano.available()) { usleep(10000); }

    std::string buffer;
    nano.read(buffer, 100);

    std::cout << "Received: " << buffer << std::endl;

    // Close the serial port
    nano.close();

    return EXIT_SUCCESS;
}
