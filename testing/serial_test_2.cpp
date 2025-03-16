#include <string>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <nlohmann/json.hpp>

#include "drivers/serial.hpp"


using json = nlohmann::json;


int main()
{
    serial::SimpleSerial nano("/dev/ttyUSB0");
    nano.begin(9600);

    usleep(2000000);

    std::cout << "started" << std::endl;

    // request laser range
    json request = {
        {"type", 2}
    };

    // send request to nano
    // nano.write(request.dump() + '\0');
    // nano.write("{\"type\": 2}\0");
    // request.dump().push_back('\0')
    // nano.write();
    nano.write_json(request);

    std::cout << "sent message: " << request.dump().c_str() << std::endl;

    // std::string readData;
    for (;;)
    {
        // wait for reply
        if (nano.available())
        {
            std::cout << "available" << std::endl;
            // nano.read(readData);

            // convert data to json
            // std::cout << "got from nano: " << readData << std::endl;

            json reply = nano.read_json();

            // json reply = json::parse(readData);
            std::cout << "laser request data:" << reply << std::endl;

            return 0;
        }

        // wait for 10ms
        usleep(1000000);
    }
}
