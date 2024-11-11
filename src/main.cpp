#include <iostream>
#include <gpiod.h>
#include <unistd.h>
#include <thread>

#include "drivers/stepper.hpp"
#include "pinout.hpp"


#define CHIP_NAME "gpiochip3"

// stepper gearing: 16:105 (1 turret rotation = 6.5625 stepper rotations = 26 880 steps)

int main()
{
    // setup chips and pins
    auto stepper0_chip = get_chip(STEPPER0_CHIP);
    auto stepper1_chip = get_chip(STEPPER1_CHIP);
    auto stepper2_chip = get_chip(STEPPER2_CHIP);

    // input order: 1, 4, 3, 2 (cheap chinese steppers ...)
    auto stepper0_a = get_pin(stepper0_chip, STEPPER0_IN1, false, false);  // in 1
    auto stepper0_b = get_pin(stepper0_chip, STEPPER0_IN4, false, false);  // in 4
    auto stepper0_c = get_pin(stepper0_chip, STEPPER0_IN3, false, false);  // in 3
    auto stepper0_d = get_pin(stepper0_chip, STEPPER0_IN2, false, false);  // in 2

    auto stepper1_a = get_pin(stepper1_chip, STEPPER1_IN1, false, false);  // in 1
    auto stepper1_b = get_pin(stepper1_chip, STEPPER1_IN4, false, false);  // in 4
    auto stepper1_c = get_pin(stepper1_chip, STEPPER1_IN3, false, false);  // in 3
    auto stepper1_d = get_pin(stepper1_chip, STEPPER1_IN2, false, false);  // in 2

    auto stepper2_a = get_pin(stepper2_chip, STEPPER2_IN1, false, false);  // in 1
    auto stepper2_b = get_pin(stepper2_chip, STEPPER2_IN4, false, false);  // in 4
    auto stepper2_c = get_pin(stepper2_chip, STEPPER2_IN3, false, false);  // in 3
    auto stepper2_d = get_pin(stepper2_chip, STEPPER2_IN2, false, false);  // in 2

    auto esl = get_pin(stepper0_chip, END_SWITCH_LEFT, true);
    auto esr = get_pin(stepper0_chip, END_SWITCH_RIGHT, true);

    auto esu = get_pin(stepper0_chip, END_SWITCH_UP, true);
    auto esd = get_pin(stepper0_chip, END_SWITCH_DOWN, true);

    // initialize stepper drivers
    stepper::Horizontal horizontal_stepper(
        {stepper0_a, stepper0_b, stepper0_c, stepper0_d},
        esr,
        esl
    );
    stepper::Vertical vertical_stepper(
        {stepper1_a, stepper1_b, stepper1_c, stepper1_d},
        {stepper2_a, stepper2_b, stepper2_c, stepper2_d},
        esu,
        esd
    );

    // calibrate steppers
    std::thread thread_obj(&stepper::Vertical::calibrate, &vertical_stepper);
    thread_obj.detach();

    horizontal_stepper.calibrate();

    // make sure all steppers are aligend
    if (thread_obj.joinable())
        thread_obj.join();

    // set speeds
    horizontal_stepper.set_speed(200);
    vertical_stepper.set_speed(100);

    // set steppers to 0
    std::thread thread_obj(&stepper::Vertical::move_absolute_angle, &vertical_stepper, 0);
    thread_obj.detach();

    horizontal_stepper.move_absolute_angle(0);

    // make sure all steppers have moved
    if (thread_obj.joinable())
        thread_obj.join();

    // basic manouvers
    std::cout << "at 0°. steps: " << horizontal_stepper.get_current_step() << std::endl;
    usleep(5000000);

    std::cout << "30°" << std::endl;
    horizontal_stepper.move_absolute_angle(30);
    usleep(5000000);


    std::cout << "100" << std::endl;
    horizontal_stepper.move_steps(100);

    usleep(5000000);


    std::cout << "-60°" << std::endl;
    horizontal_stepper.move_relative_angle(-60);
    usleep(5000000);

    horizontal_stepper.move_absolute_angle(0);

    // cleanup
    horizontal_stepper.off();
    usleep(100000);

    // release lines and chips
    gpiod_line_release(stepper0_a);
    gpiod_line_release(stepper0_b);
    gpiod_line_release(stepper0_c);
    gpiod_line_release(stepper0_d);

    gpiod_line_release(stepper1_a);
    gpiod_line_release(stepper1_b);
    gpiod_line_release(stepper1_c);
    gpiod_line_release(stepper1_d);

    gpiod_line_release(stepper2_a);
    gpiod_line_release(stepper2_b);
    gpiod_line_release(stepper2_c);
    gpiod_line_release(stepper2_d);

    gpiod_chip_close(stepper0_chip);
    gpiod_chip_close(stepper1_chip);
    gpiod_chip_close(stepper2_chip);

    return 0;
}
