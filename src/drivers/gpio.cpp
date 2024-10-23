#include <iostream>
#include "gpio.hpp"


gpiod_line* get_pin(gpiod_chip* chip, unsigned int pin, bool is_input, bool default_value)
{
    gpiod_line* line = gpiod_chip_get_line(chip, pin);
    if (!line)
    {
        std::cerr << "Failed to get line" << std::endl;
        return nullptr;
    }

    if (is_input)
    {
        if (gpiod_line_request_input(line, "Fridrich Turret") < 0)
        {
            std::cerr << "failed to set to input" << std::endl;
            return nullptr;
        }
    }
    else
    {
        if (gpiod_line_request_output(line, "Fridrich Turret", default_value) < 0)
        {
            std::cerr << "failed to set to output" << std::endl;
            return nullptr;
        }
    }

    std::cout << "set " << pin << " to " << (is_input ? "input" : "output") << std::endl;

    return line;
}
