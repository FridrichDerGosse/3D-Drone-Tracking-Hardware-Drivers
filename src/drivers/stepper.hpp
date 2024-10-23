/**
 * @file stepper.hpp
 * @author Nilusink
 * @brief stepper drivers for the dipl. turret
 * @version 1.0
 * @date 2024-10-22
 *
 */
#pragma once

#include "helper.hpp"
#include "gpio.hpp"


namespace stepper {
    typedef struct
    {
        pin_t pin_a;
        pin_t pin_b;
        pin_t pin_c;
        pin_t pin_d;
    } stepper_pinout_t;

    extern const int sequence[8];

    class Base {
        private:
            int16_t current_step = 0;

        protected:
            const uint8_t sequence_size = 8;
            const uint16_t steps_per_rev = 4096;  // I think, not sure tho
            uint16_t step_delay_us = 1000;
            stepper_pinout_t pins;

            void update_pins() const;
            bool can_move() const; // later used for end-switches

        public:
            explicit Base(stepper_pinout_t pins);

            int16_t get_current_step() const;

            void set_speed(uint8_t speed);

            int8_t move_steps(signed short int n);
            void off() const;
    };

    class Horizontal : public Base
    {
        private:
            uint16_t n_steps = 0;  // set by calibration, n steps between end switches
            int16_t max_step_left = 0;
            int16_t max_step_right = 0;
            bool is_calibrating = false;

        protected:
            const uint8_t angle_size = 120;

            pin_t end_left_pin;
            pin_t end_right_pin;

            // internal functions
            bool check_calibrated() const;
            bool can_move() const; // later used for end-switches

        public:
            Horizontal(
                stepper_pinout_t pins,
                pin_t end_left,
                pin_t end_right
            );

            /**
             *
             * @return current angle of the stepper
             */
            [[nodiscard]] double get_current_angle() const;

            int8_t move_steps(int16_t n);
            void move_relative_angle(double angle_delta);
            void move_absolute_angle(double angle);
            int8_t calibrate();
    };
};
