#include <iostream>
#include <gpiod.h>
#include <unistd.h>

#include "drivers/stepper.hpp"


#define CHIP_NAME "gpiochip3"
#define LINE_NUMBER B+3

int main()
{
    gpiod_chip* chip_3 = gpiod_chip_open_by_name(CHIP_NAME);
    if (!chip_3)
    {
        std::cerr << "Failed to open chip" << std::endl;
        return -1;
    }

    gpiod_line* in1 = get_pin(chip_3, B+3, false, false);
    gpiod_line* in2 = get_pin(chip_3, B+2, false, false);
    gpiod_line* in3 = get_pin(chip_3, B+1, false, false);
    gpiod_line* in4 = get_pin(chip_3, B+6, false, false);

    gpiod_line* esl = get_pin(chip_3, A+7, true);
    gpiod_line* esr = get_pin(chip_3, C+2, true);

    // for (;;)
    // {
    //     std::cout << "Left: " << gpiod_line_get_value(esl) << ", Right: " << gpiod_line_get_value(esr) << std::endl;
    //     usleep(100000);
    // }

    stepper::Base simple_stepper({in1, in2, in3, in4});
    stepper::Horizontal stepper({in1, in2, in3, in4}, esl, esr);
    stepper.set_speed(200);

    stepper.calibrate();

    // std::cout << "moving direction 1" << std::endl;
    // std::cout << "speed 50" << std::endl;
    // simple_stepper.set_speed(50);
    // simple_stepper.move_steps(500);

    // std::cout << "speed 100" << std::endl;
    // simple_stepper.set_speed(100);
    // simple_stepper.move_steps(700);

    // std::cout << "speed 150" << std::endl;
    // simple_stepper.set_speed(150);
    // simple_stepper.move_steps(1000);

    // std::cout << "speed 200" << std::endl;
    // simple_stepper.set_speed(200);
    // simple_stepper.move_steps(1000);

    // std::cout << "speed 250" << std::endl;
    // simple_stepper.set_speed(250);
    // simple_stepper.move_steps(1000);

    // usleep(1000000);

    // std::cout << "moving direction 2" << std::endl;
    // simple_stepper.move_steps(-4200);

    // simple_stepper.off();

    // cleanup
    stepper.off();
    usleep(100000);

    gpiod_line_release(in1);
    gpiod_line_release(in2);
    gpiod_line_release(in3);
    gpiod_line_release(in4);
    gpiod_chip_close(chip_3);

    return 0;
}
