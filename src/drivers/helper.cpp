#include "helper.hpp"
#include <iostream>


void pin_write(uint8_t pin, bool state) {
    std::cout << "Pin " << static_cast<unsigned short int>(pin) << " was set to " << state << std::endl;
}

bool pin_read(uint8_t pin) {
    std::cout << "Pin " << pin << " was read " << std::endl;
    return pin;
}


double map(double value, double in_min, double in_max, double out_min, double out_max)
{
    return (value - in_min) * (out_max - out_min) / in_max + out_min;
}
