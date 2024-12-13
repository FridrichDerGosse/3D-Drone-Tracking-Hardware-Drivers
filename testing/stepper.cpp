#include <iostream>
#include <gpiod.h>
#include <unistd.h>
#include <thread>
#include <memory>

#include "drivers/stepper.hpp"
#include "pinout.hpp"


#define CHIP_NAME "gpiochip3"

// stepper gearing: 16:105 (1 turret rotation = 6.5625 stepper rotations = 26 880 steps)

int main()
{
    // setup chips and pins
    auto gpio0 = get_chip("gpiochip0");
    // auto gpio1 = get_chip("gpiochip1");
    auto yaw_chip = get_chip(YAW_CHIP);
    auto pitch_chip = get_chip(PITCH_CHIP);

    // initialize stepper pins
    // input order: 1, 4, 3, 2 (cheap chinese steppers ...)
    auto yaw_a = get_pin(yaw_chip, YAW_IN1, false, false);  // in 1
    auto yaw_b = get_pin(yaw_chip, YAW_IN4, false, false);  // in 4
    auto yaw_c = get_pin(yaw_chip, YAW_IN3, false, false);  // in 3
    auto yaw_d = get_pin(yaw_chip, YAW_IN2, false, false);  // in 2

    auto pitch_left_a = get_pin(pitch_chip, PITCH_LEFT_IN1, false, false);  // in 1
    auto pitch_left_b = get_pin(pitch_chip, PITCH_LEFT_IN4, false, false);  // in 4
    auto pitch_left_c = get_pin(pitch_chip, PITCH_LEFT_IN3, false, false);  // in 3
    auto pitch_left_d = get_pin(pitch_chip, PITCH_LEFT_IN2, false, false);  // in 2

    auto pitch_right_a = get_pin(pitch_chip, PITCH_RIGHT_IN1, false, false);  // in 1
    auto pitch_right_b = get_pin(pitch_chip, PITCH_RIGHT_IN4, false, false);  // in 4
    auto pitch_right_c = get_pin(pitch_chip, PITCH_RIGHT_IN3, false, false);  // in 3
    auto pitch_right_d = get_pin(pitch_chip, PITCH_RIGHT_IN2, false, false);  // in 2

    // initialize end switch pins
    auto eslr = get_pin(gpio0, END_SWITCH_LR, true);

    auto esu = get_pin(pitch_chip, END_SWITCH_UP, true);
    auto esd = get_pin(pitch_chip, END_SWITCH_DOWN, true);

    // create pin groups
    stepper::stepper_pinout_t horizontal_pins = {yaw_a, yaw_b, yaw_c, yaw_d};
    stepper::stepper_pinout_t vertical_left_pins = {pitch_left_a, pitch_left_b, pitch_left_c, pitch_left_d};
    stepper::stepper_pinout_t vertical_right_pins = {pitch_right_a, pitch_right_b, pitch_right_c, pitch_right_d};

    // initialize stepper drivers
    stepper::Horizontal horizontal_stepper(
        horizontal_pins,
        eslr
    );

    stepper::Vertical vstepper(
        vertical_left_pins,
        vertical_right_pins,
        esu,
        esd
    );

    // calibrate steppers
    std::thread tmp_thread(&stepper::Vertical::calibrate, &vstepper);
    horizontal_stepper.calibrate();

    // make sure all steppers are aligend
    if (tmp_thread.joinable())
        tmp_thread.join();

    std::cout << "alignment done" << std::endl;

    // set speeds
    horizontal_stepper.set_speed(230);
    vstepper.set_speed(10);

    // set steppers to 0
    tmp_thread = std::thread(&stepper::Vertical::move_absolute_angle, &vstepper, 0);
    horizontal_stepper.move_absolute_angle(0);

    // make sure all steppers have moved
    if (tmp_thread.joinable())
        tmp_thread.join();

    // wait five seconds after homing
    usleep(5000000);

    // // basic manouvers
    tmp_thread = std::thread(&stepper::Vertical::move_absolute_angle, &vstepper, 30);
    horizontal_stepper.move_absolute_angle(60);

    // make sure all steppers have moved
    if (tmp_thread.joinable())
        tmp_thread.join();

    tmp_thread = std::thread(&stepper::Vertical::move_relative_angle, &vstepper, 20);
    horizontal_stepper.move_relative_angle(-120);

    // make sure all steppers have moved
    if (tmp_thread.joinable())
        tmp_thread.join();

    vstepper.set_speed(200);

    tmp_thread = std::thread(&stepper::Vertical::move_absolute_angle, &vstepper, 0);
    horizontal_stepper.move_absolute_angle(0);

    // make sure all steppers have moved
    if (tmp_thread.joinable())
        tmp_thread.join();

    // cleanup
    all_off(horizontal_stepper, vstepper);

    // release lines and chips
    all_shut(horizontal_stepper, vstepper);

    gpiod_chip_close(gpio0);
    // gpiod_chip_close(gpio1);
    gpiod_chip_close(yaw_chip);
    gpiod_chip_close(pitch_chip);

    return 0;
}
