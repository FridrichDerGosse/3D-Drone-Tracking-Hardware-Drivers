/**
 * @file main.cpp
 * @author Nilusink
 * @brief manual alignment
 * @date 2024-12-04
 * 
 * @copyright Copyright Nilusink (c) 2024
 * 
 */
#include <iostream>
#include <gpiod.h>
#include <unistd.h>
#include <thread>
#include <chrono>
#include <atomic>
#include <memory>

#include "drivers/stepper.hpp"
#include "drivers/nano.hpp"
#include "pinout.hpp"


void alignment_countdown(std::atomic<bool>& finished_flag)
{
    // set starting time
    auto start = std::chrono::high_resolution_clock::now();

    
    // wait for finished flag to be set
    std::chrono::microseconds delta;
    std::chrono::_V2::system_clock::time_point now;
    while (!finished_flag)
    {
        now = std::chrono::high_resolution_clock::now();
        delta = std::chrono::duration_cast<std::chrono::microseconds>(now - start);

        // count down from 70 (1 minute and 10 seconds)
        std::cout << "\rtime left: " << std::setw(4) << std::setfill(' ') << std::left << std::fixed << std::setprecision(1) << (70.f - delta.count() / 1000000.f) << " seconds" << std::flush;

        // wait 100 ms
        usleep(100000);
    }

    // print total alignment time
    std::cout << "\rAlignment took " << std::fixed << std::setprecision(2) << (delta.count() / 1000000.f) << " seconds" << std::endl;
}


// TODO: Messages are still delayed once (don't ask me how)


int main()
{
    // serial setup
    nano::Nano my_nano("/dev/ttyUSB0");
    my_nano.begin(9600);

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

    // turn laser off bevore alignment, also wait 2 seconds for nano to start
    // usleep(2000000);
    // my_nano.set_laser_state(0);

    // calibrate steppers
    std::cout << "calibrating turret ..." << std::endl;

    // countdown thread
    std::atomic<bool> alignment_flag(false);
    std::thread clock_thread(alignment_countdown, std::ref(alignment_flag));

    // horizontal stepper alignment
    std::thread tmp_thread(&stepper::Horizontal::calibrate, &horizontal_stepper);

    // wait two seconds for vertical to start
    vstepper.calibrate();

    // make sure all steppers are aligend
    if (tmp_thread.joinable())
        tmp_thread.join();

    // finish clock thread
    alignment_flag = true;
    if (clock_thread.joinable())
        clock_thread.join();

    std::cout << "homing turret ..." << std::endl;

    // set speeds
    horizontal_stepper.set_speed(230);
    vstepper.set_speed(200);

    // home turret
    // set steppers to 0
    tmp_thread = std::thread(&stepper::Vertical::move_absolute_angle, &vstepper, 0);
    horizontal_stepper.move_absolute_angle(0);

    // make sure all steppers have moved
    if (tmp_thread.joinable())
        tmp_thread.join();

    std::cout << "syntax: <direction (h,v for absolute): u,d,l,r> <ammount in °>" << std::endl;
    std::cout << "alternatively, input: " << std::endl;
    std::cout << "\t* \"m\": measure" << std::endl;
    std::cout << "\t* \"c\": confirm" << std::endl;
    std::cout << "\t* \"e\": exit" << std::endl;
    std::cout << "\t* \"f\": fan speed (0 to 255)" << std::endl;
    std::cout << "\t* \"t\": TOF laser on/off (1/0)" << std::endl;

    char direction;
    double degrees;
    bool running = true;
    while (running)
    {
        std::cout << ">> ";
        std::cin >> direction >> degrees;

        switch (direction)
        {
        case 'u':
            std::cout << "moving \"" << direction << "\" by " << degrees << "°" << std::endl;
            vstepper.move_relative_angle(degrees);
            break;

        case 'd':
            std::cout << "moving \"" << direction << "\" by " << degrees << "°" << std::endl;
            vstepper.move_relative_angle(-degrees);
            break;

        case 'l':
            std::cout << "moving \"" << direction << "\" by " << degrees << "°" << std::endl;
            horizontal_stepper.move_relative_angle(-degrees);
            break;

        case 'r':
            std::cout << "moving \"" << direction << "\" by " << degrees << "°" << std::endl;
            horizontal_stepper.move_relative_angle(degrees);
            break;

        case 'h':
            std::cout << "moving \"" << direction << "\" by " << degrees << "°" << std::endl;
            horizontal_stepper.move_absolute_angle(degrees);
            break;

        case 'v':
            std::cout << "moving \"" << direction << "\" by " << degrees << "°" << std::endl;
            vstepper.move_absolute_angle(degrees);
            break;

        case 'c':
        {
            std::cout << "measuring ..." << std::endl;

            // request laser measurement
            double distance = my_nano.laser_measure();

            if (distance < 0)
            {
                std::cout << "tof measurement failure" << std::endl;
                break;
            }

            std::cout << "position: ";
            std::cout << std::fixed << std::setprecision(3) << vstepper.get_current_angle() << "°v, ";
            std::cout << std::fixed << std::setprecision(3) << horizontal_stepper.get_current_angle() << "°h at ";
            std::cout << std::fixed << std::setprecision(3) << distance << "m locked" << std::endl;
            std::cout << "exiting" << std::endl;
            running = false;
            break;
        }

        case 'm':
        {
            std::cout << "measuring ..." << std::endl;

            // request laser measurement
            double distance = my_nano.laser_measure();

            if (distance < 0)
            {
                std::cout << "tof measurement failure" << std::endl;
                break;
            }

            std::cout << "position: ";
            std::cout << std::fixed << std::setprecision(3) << vstepper.get_current_angle() << "°v, ";
            std::cout << std::fixed << std::setprecision(3) << horizontal_stepper.get_current_angle() << "°h at ";
            std::cout << std::fixed << std::setprecision(3) << distance << "m measured" << std::endl;

            // turn laser back on
            // my_nano.set_laser_state(1);
            break;
        }

        case 'f':
        {
            uint8_t fan_speed = degrees;
            std::cout << "Setting fan to: " << (int)fan_speed << std::endl;

            // set fan speed and wait for answer
            my_nano.set_fan_speed(fan_speed);
            std::cout << "done" << std::endl;

            break;
        }

        case 't':
        {
            bool state = direction == 0;
            std::cout << "Setting TOF laser to: " << (state ? "on" : "off") << std::endl;
            std::cout << "Setting " << (my_nano.set_laser_state(state) ? "success": "fail") << std::endl;
            break;
        }

        case 'e':
            std::cout << "exiting" << std::endl;
            running = false;
            break;

        default:
            std::cout << "invalid" << std::endl;
            break;
        }
        std::cin.clear();
    }

    all_off(horizontal_stepper, vstepper);
    all_shut(horizontal_stepper, vstepper);

    return 0;
}