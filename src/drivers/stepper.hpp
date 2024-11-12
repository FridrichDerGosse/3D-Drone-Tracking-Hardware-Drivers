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
            int16_t current_step = 1;

        protected:
            const uint8_t sequence_size = 8;
            const uint16_t steps_per_rev = 26880;  // (4096 * gearing) I think, not sure tho
            uint16_t step_delay_us = 1000;
            stepper_pinout_t pins;

            void update_pins() const;
            bool can_move() const; // later used for end-switches

        public:
            explicit Base(stepper_pinout_t pins);

            /**
             * @brief getter for current_Step
             * 
             * @return int16_t current step delta from start
             */
            int16_t get_current_step() const;

            /**
             * @brief set how fast the stepper should move
             * 
             * @param speed 0 to 250
             */
            void set_speed(uint8_t speed);

            /**
             * @brief move the stepper motor
             * 
             * @param n how many steps to move
             * @return 0: OK, 4: can't move
             */
            int8_t move_steps(int16_t n);

            /**
             * @brief turn all stepper pins off (relese motor)
             * 
             */
            void off() const;

            /**
             * @brief shutdown all pins
             * 
             */
            void shutdown() const;
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
            bool can_move() const; // later used for end-switches

        public:
            Horizontal(
                stepper_pinout_t pins,
                pin_t end_left,
                pin_t end_right
            );

            int8_t calibrate();
            bool check_calibrated() const;

            /**
             * @brief get the current angle in degrees
             * 
             * @return double
             */
            double get_current_angle() const;

            /**
             * @brief move the stepper motor
             * 
             * @param n how many steps to move
             * @return 0: OK, 1: not calibrated, 2: max left, 3: max right, 4: can't move, 5: end_switch while moving
             */
            int8_t move_steps(int16_t n);

            /**
             * @brief move by a specified amount of degrees
             * 
             * @param angle_delta move by
             */
            void move_relative_angle(double angle_delta);

            /**
             * @brief move to an absolute position
             * 
             * @param angle move to
             */
            void move_absolute_angle(double angle);
    };

    class Vertical
    {
        private:
            int16_t current_step = 1;
            uint16_t n_steps = 0;  // set by calibration, n steps between end switches
            int16_t max_step_up = 0;
            int16_t max_step_down = 0;
            bool is_calibrating = false;
        
        protected:
            const uint8_t sequence_size = 8;
            const uint16_t steps_per_rev = 26880;  // I think, not sure tho
            uint16_t step_delay_us = 1000;

            const int8_t max_down_angle = -20;
            const int8_t max_up_angle = 60;
            const uint8_t angle_size = max_up_angle - max_down_angle;

            pin_t end_up_pin;
            pin_t end_down_pin;

            stepper_pinout_t pins_left;
            stepper_pinout_t pins_right;

            // internal functions
            void update_pins() const;
            bool can_move() const; // later used for end-switches
        
        public:
            Vertical(
                stepper_pinout_t pins_left,
                stepper_pinout_t pins_right,
                pin_t end_up,
                pin_t end_down
            );

            int8_t calibrate();
            bool check_calibrated() const;

            /**
             * @brief set how fast the stepper should move
             * 
             * @param speed 0 to 250
             */
            void set_speed(uint8_t speed);


            /**
             * @brief getter for current_Step
             * 
             * @return int16_t current step delta from start
             */
            int16_t get_current_step() const;

            /**
             * @brief move the stepper motor
             * 
             * @param n how many steps to move
             * @return 0: OK, 1: not calibrated, 2: max left, 3: max right, 4: can't move, 5: end_switch while moving
             */
            int8_t move_steps(int16_t n);

            /**
             * @brief get the current angle in degrees
             * 
             * @return double
             */
            double get_current_angle() const;

            /**
             * @brief move by a specified amount of degrees
             * 
             * @param angle_delta move by
             */
            void move_relative_angle(double angle_delta);

            /**
             * @brief move to an absolute position
             * 
             * @param angle move to
             */
            void move_absolute_angle(double angle);

            /**
             * @brief turn all stepper pins off (relese motor)
             * 
             */
            void off() const;

            /**
             * @brief shutdown all pins
             * 
             */
            void shutdown() const;
    };

    // shared functions
    void home_all(Horizontal &hor, Vertical &ver);
    void all_off(Horizontal &hor, Vertical &ver);
    void all_shut(Horizontal &hor, Vertical &ver);
};
