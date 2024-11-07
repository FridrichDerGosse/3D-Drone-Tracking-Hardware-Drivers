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

    // input order: 1, 4, 3, 2 (cheap chinese steppers ...)
    gpiod_line* a = get_pin(chip_3, B+3, false, false);  // in 1
    gpiod_line* b = get_pin(chip_3, B+6, false, false);  // in 4
    gpiod_line* c = get_pin(chip_3, B+1, false, false);  // in 3
    gpiod_line* d = get_pin(chip_3, B+2, false, false);  // in 2

    gpiod_line* esl = get_pin(chip_3, A+7, true);
    gpiod_line* esr = get_pin(chip_3, C+2, true);

    // for (;;)
    // {
    //     std::cout << "Left: " << gpiod_line_get_value(esl) << ", Right: " << gpiod_line_get_value(esr) << std::endl;
    //     usleep(100000);
    // }

    stepper::Base simple_stepper({a, b, c, d});
    stepper::Horizontal stepper({a, b, c, d}, esr, esl);
    stepper.set_speed(100);

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
    std::cout << "at 0°. steps: " << stepper.get_current_step() << std::endl;
    usleep(5000000);

    std::cout << "30°" << std::endl;
    stepper.move_absolute_angle(30);
    usleep(5000000);


    std::cout << "100" << std::endl;
    stepper.move_steps(100);

    usleep(5000000);


    std::cout << "-60°" << std::endl;
    stepper.move_relative_angle(-60);
    usleep(5000000);

    // cleanup
    stepper.off();
    usleep(100000);

    gpiod_line_release(a);
    gpiod_line_release(b);
    gpiod_line_release(c);
    gpiod_line_release(d);
    gpiod_chip_close(chip_3);

    return 0;
}
