#include <string>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>


using json = nlohmann::json;


int main()
{
    std::string str; 
    std::fstream f; 

    f.open("/dev/ttyUSB0"); 

    std::cout << "started" << std::endl;

    // request a "send"
    json request = {
        {"type", 0},
        {"data", "asdlkfasjdf"}
    };

    // send request to nano
    f << request.dump();

    for (;;)
    {
        // receive response
        while (f >> str)
        {
            std::cout << "nano: \"" << str << "\"";
        }
    }
}
