#include <iostream>
#include "gpio.hpp"


gpiod_line* get_pin(gpiod_chip* chip, unsigned int pin, bool is_input, bool default_value)
{
    // get line from chip
    gpiod_line* line = gpiod_chip_get_line(chip, pin);
    if (!line)
    {
        std::cerr << "Failed to get line" << std::endl;
        return nullptr;
    }

    // define line a input / output
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

    return line;
}

// implementation for gpiod
int pin_write(pin_t pin, bool value)
{
    return gpiod_line_set_value(pin, value);
}

bool pin_read(pin_t pin)
{
    return gpiod_line_get_value(pin);
}
