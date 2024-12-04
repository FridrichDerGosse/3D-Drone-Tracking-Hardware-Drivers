#include <string>
#include <fstream>
#include <iostream>


int main()
{
    std::string str; 
    std::fstream f; 

    f.open("/dev/ttyUSB0"); 

    std::cout << "hiii" << std::endl;

    for (;;)
    {
        while (f >> str)
        {
            std::cout << str;
        }
    }
}
