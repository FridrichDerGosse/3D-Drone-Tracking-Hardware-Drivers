#include "stepper.hpp"

#include <iostream>
#include <unistd.h>
#include <algorithm>
#include <thread>

using namespace stepper;


// uln2003 step pattern
const int stepper::sequence[8] = {
    0b1001,
    0b1000,
    0b1100,
    0b0100,
    0b0110,
    0b0010,
    0b0011,
    0b0001
};


// Basic stepper
void Base::update_pins() const
{
    // update all pins corresponding to the current sequence position
    pin_write(pins.pin_a, (sequence[(uint16_t)current_step % sequence_size] & 0b0001));
    pin_write(pins.pin_b, (sequence[(uint16_t)current_step % sequence_size] & 0b0010));
    pin_write(pins.pin_c, (sequence[(uint16_t)current_step % sequence_size] & 0b0100));
    pin_write(pins.pin_d, (sequence[(uint16_t)current_step % sequence_size] & 0b1000));
}

bool Base::can_move() const
{
    return true;
}

Base::Base(const stepper_pinout_t pins)
    : pins(pins)
{
    // turn motor on
    update_pins();
}

int16_t Base::get_current_step() const
{
    return current_step;
}

void Base::set_speed(uint8_t speed)
{
    // map speed to delay (max 250 because idk why but it won't work if i don't do this)
    step_delay_us = map(std::min((int)speed, 250), 0, 255, 10000, 400);
}

int8_t Base::move_steps(int16_t n) {
    // check if movement is allowed
    if (!can_move())
    {
        if (warnings)
            std::cout << "can't move" << std::endl;
    
        return 4;
    }

    // step in one direction
    for (signed short int i = 0; i < n; i++) {
        current_step += 1;
        update_pins();

        if (!can_move())
        {
            if (warnings)
                std::cout << "hit end switch" << std::endl;
        
            return 5;
        }

        // stepper speed
        usleep(step_delay_us);
    };

    // step in the other direction
    for (signed short int i = 0; i > n; i--) {
        current_step -= 1;
        update_pins();

        if (!can_move())
        {
            if (warnings)
                std::cout << "hit end switch" << std::endl;
        
            return 5;
        }

        // stepper speed
        usleep(step_delay_us);
    };

    return 0;
};

void Base::off() const
{
    pin_write(pins.pin_a, false);
    pin_write(pins.pin_b, false);
    pin_write(pins.pin_c, false);
    pin_write(pins.pin_d, false);
};

void Base::shutdown() const
{
    cleanup_pin(pins.pin_a);
    cleanup_pin(pins.pin_b);
    cleanup_pin(pins.pin_c);
    cleanup_pin(pins.pin_d);
}

// Horizontal Stepper
bool Horizontal::check_calibrated() const
{
    return n_steps != 0;
};

bool Horizontal::can_move() const
{
    // "override" end-switches for calibration
    if (is_calibrating)
        return true;

    return !pin_read(end_switch_pin);
}

Horizontal::Horizontal(
    const stepper_pinout_t pins,
    const pin_t end_switch
)
    : Base(pins), end_switch_pin(end_switch)
{};

double Horizontal::get_current_angle() const
{
    if (debugging)
    {
        std::cout << "current step: " << get_current_step() << std::endl;
        std::cout << "max left: " << max_step_left << ", right: " << max_step_right << std::endl;
    }

    // convert step position to angular position
    return map(
        get_current_step(),
        max_step_right,
        max_step_left,
        -angle_size / 2,
        angle_size / 2
    );
};

int8_t Horizontal::calibrate() {
    // set to calibration mode so the stepper is allowed to move
    is_calibrating = true;

    // set a maximum of one rotation to find the end-switch
    max_step_left = Base::steps_per_rev;
    max_step_right = -Base::steps_per_rev;

    uint16_t old_delay = step_delay_us;

    // set speed
    set_speed(200);

    // move left until hitting the end switch
    while (!pin_read(end_switch_pin))
    {
        if (move_steps(1) > 0)
        {
            if (warnings)
                std::cerr << "error aligning: stepper can't find end switch" << std::endl;
    
            return 1;
        }
    }

    max_step_left = get_current_step();

    // move right to just bevore the end switch
    // end to end should be about 16497 steps
    set_speed(200);
    move_steps(-200);
    set_speed(230);
    move_steps(-100);
    set_speed(240);
    move_steps(-400);
    set_speed(240);
    move_steps(-15000);
    set_speed(200);
    move_steps(-200);
    set_speed(150);
    move_steps(-200);

    // // only for non-endswitch alignment
    // move_steps(-400);

    set_speed(100);

    // move right until hitting the end switch
    while (!pin_read(end_switch_pin))
    {
        if (move_steps(-1) > 0)
        {
            if (warnings)
                std::cerr << "error aligning: stepper can't find end switch" << std::endl;
    
            return 1;
        }
    }

    max_step_right = get_current_step();

    // calculate fancy stuff
    n_steps = max_step_left - max_step_right;

    if (debugging)
    {
        std::cout << "current step: " << get_current_step() << std::endl;
        std::cout << "max left: " << max_step_left << ", right: " << max_step_right << std::endl;
        std::cout << "calibrated: angle=" << (int)angle_size << "°, steps=" << n_steps << " with " << ((double)angle_size / n_steps) << "° per step" << std::endl;
    }

    set_speed(150);
    move_steps(100);

    step_delay_us = old_delay;
    is_calibrating = false;

    return 0;
}

int8_t Horizontal::move_steps(int16_t n) {
    if (!check_calibrated() && !is_calibrating)
    {
        if (warnings)
            std::cerr << "can't move, not calibrated yet!" << std::endl;
    
        return 1;
    }

    if (get_current_step() + n > max_step_left)
    {
        if (warnings)
            std::cerr << "steps out of moveable range, moving to max left" << std::endl;
        n = max_step_left - get_current_step();
        return 2;
    }

    if (get_current_step() + n < max_step_right)
    {
        if (warnings)
            std::cerr << "steps out of moveable range, moving to max right" << std::endl;
        n = max_step_right - get_current_step();
        return 3;
    }

    if (debugging)
        std::cout << "moving by " << n << " steps. Current step:" << get_current_step() << std::endl;

    return Base::move_steps(n);
};

void Horizontal::move_relative_angle(const double angle_delta)
{
    if (debugging)
        std::cout << "moving by " << angle_delta << "° / " << ((double)angle_size / n_steps) << " degrees per step = " << (angle_delta / ((double)angle_size / n_steps)) << std::endl;
    
    move_steps(angle_delta / ((double)angle_size / n_steps));
};

void Horizontal::move_absolute_angle(const double angle)
{
    double angle_delta = angle - get_current_angle();

    if (debugging)
        std::cout << "moving to " << angle << "°: current angle=" << get_current_angle() << "°, delta: " << angle_delta << "°" << std::endl;
    
    move_relative_angle(angle_delta);
};


// -------------------------------------- Vertical
void Vertical::update_pins() const
{
    // two stepper with reversed direction
    uint8_t sequence_pos_p = (uint16_t)current_step % sequence_size;
    uint8_t sequence_pos_n = sequence_size - (sequence_pos_p + 1);

    // update all pins corresponding to the current sequence position
    pin_write(pins_left.pin_a, (sequence[sequence_pos_p] & 0b0001));
    pin_write(pins_left.pin_b, (sequence[sequence_pos_p] & 0b0010));
    pin_write(pins_left.pin_c, (sequence[sequence_pos_p] & 0b0100));
    pin_write(pins_left.pin_d, (sequence[sequence_pos_p] & 0b1000));

    pin_write(pins_right.pin_a, (sequence[sequence_pos_n] & 0b0001));
    pin_write(pins_right.pin_b, (sequence[sequence_pos_n] & 0b0010));
    pin_write(pins_right.pin_c, (sequence[sequence_pos_n] & 0b0100));
    pin_write(pins_right.pin_d, (sequence[sequence_pos_n] & 0b1000));
}

bool Vertical::check_calibrated() const
{
    return n_steps != 0;
};

bool Vertical::can_move() const
{
    // "override" end-switches for calibration
    if (is_calibrating)
        return true;

    return !pin_read(end_up_pin) and !pin_read(end_down_pin);
}

Vertical::Vertical(
    stepper_pinout_t pins_left,
    stepper_pinout_t pins_right,
    pin_t end_up,
    pin_t end_down
)
 : pins_left(pins_left), pins_right(pins_right), end_up_pin(end_up), end_down_pin(end_down)
{
    update_pins();
};

int8_t Vertical::calibrate() {
    // set to calibration mode so the stepper is allowed to move
    is_calibrating = true;

    // set a maximum of one rotation to find the end-switch
    max_step_up = steps_per_rev;
    max_step_down = -steps_per_rev;

    uint16_t old_delay = step_delay_us;

    // set speed
    set_speed(50);

    // move up until hitting the end switch
    while (!pin_read(end_up_pin))
    {
        if (move_steps(1) > 0)
        {
            if (warnings)
                std::cerr << "error aligning: stepper can't find end switch" << std::endl;
        
            return 1;
        }
        
        if (pin_read(end_down_pin))
        {
            if (warnings)
                std::cerr << "error aligning: wrong end switch touched - check wiring" << std::endl;
            
            return 2;
        }
    }


    max_step_up = get_current_step();

    // move right to just bevore the end switch
    // set_speed(200);
    // move_steps(-2200);
    // set_speed(150);
    // move_steps(-150);
    // set_speed(100);
    // move_steps(-150);

    set_speed(50);

    // move down until hitting the end switch
    while (!pin_read(end_down_pin))
    {
        if (move_steps(-1) > 0)
        {
            if (warnings)
                std::cerr << "error aligning: stepper can't find end switch" << std::endl;
    
            return 1;
        }
    }

    max_step_down = get_current_step();

    // calculate fancy stuff
    n_steps = max_step_up - max_step_down;

    if (debugging)
        std::cout << "calibrated: angle=" << (int)angle_size << "°, steps=" << n_steps << " with " << ((double)angle_size / n_steps) << "° per step" << std::endl;

    move_steps(50);

    step_delay_us = old_delay;
    is_calibrating = false;

    return 0;
}

void Vertical::set_speed(uint8_t speed)
{
    // map speed to delay (max 250 because idk why but it won't work if i don't do this)
    step_delay_us = map(std::min((int)speed, 250), 0, 255, 10000, 400);
}


int16_t Vertical::get_current_step() const
{
    return current_step;
}

int8_t Vertical::move_steps(int16_t n)
{
    if (!check_calibrated() && !is_calibrating)
    {
        if (warnings)
            std::cerr << "can't move, not calibrated yet!" << std::endl;
    
        return 1;
    }

    if (get_current_step() + n > max_step_up)
    {
        if (warnings)
            std::cerr << "steps out of moveable range, moving to max left" << std::endl;
    
        n = max_step_up - get_current_step();
        return 2;
    }

    if (get_current_step() + n < max_step_down)
    {
        if (warnings)
            std::cerr << "steps out of moveable range, moving to max right" << std::endl;
    
        n = max_step_down - get_current_step();
        return 3;
    }

    if (debugging)
        std::cout << "moving by " << n << " steps. Current step:" << get_current_step() << std::endl;

    // check if movement is allowed
    if (!can_move())
    {
        if (warnings)
            std::cout << "can't move" << std::endl;
        
        return 4;
    }

    // step in one direction
    for (signed short int i = 0; i < n; i++) {
        current_step += 1;
        update_pins();

        if (!can_move())
        {
            if (warnings)
                std::cout << "hit end switch" << std::endl;
    
            return 5;
        }

        // stepper speed
        usleep(step_delay_us);
    };

    // step in the other direction
    for (signed short int i = 0; i > n; i--) {
        current_step -= 1;
        update_pins();

        if (!can_move())
        {
            if (warnings)
                std::cout << "hit end switch" << std::endl;

            return 5;
        }

        // stepper speed
        usleep(step_delay_us);
    };

    return 0;
}

double Vertical::get_current_angle() const
{
    // convert step position to angular position
    // std::cout << "steps: " << (int)max_step_down << ", " << (int)max_step_up << ", curr: " << get_current_step() << std::endl;
    // std::cout << "angle: " << (int)max_down_angle << ", " << (int)max_up_angle << std::endl;
    return map(
        get_current_step(),
        max_step_down,
        max_step_up,
        max_down_angle,
        max_up_angle
    );
};

void Vertical::move_relative_angle(const double angle_delta)
{
    if (debugging)
        std::cout << "moving by " << angle_delta << "° / " << ((double)angle_size / n_steps) << " degrees per step = " << (angle_delta / ((double)angle_size / n_steps)) << std::endl;
    
    move_steps(angle_delta / ((double)angle_size / n_steps));
};

void Vertical::move_absolute_angle(const double angle)
{
    double angle_delta = angle - get_current_angle();

    if (debugging)
        std::cout << "moving to " << angle << "°: current angle=" << (int)get_current_angle() << "°, delta: " << angle_delta << "°" << std::endl;
    
    move_relative_angle(angle_delta);
};

void Vertical::off() const
{
    pin_write(pins_left.pin_a, false);
    pin_write(pins_left.pin_b, false);
    pin_write(pins_left.pin_c, false);
    pin_write(pins_left.pin_d, false);

    pin_write(pins_right.pin_a, false);
    pin_write(pins_right.pin_b, false);
    pin_write(pins_right.pin_c, false);
    pin_write(pins_right.pin_d, false);
}

void Vertical::shutdown() const
{
    cleanup_pin(pins_left.pin_a);
    cleanup_pin(pins_left.pin_b);
    cleanup_pin(pins_left.pin_c);
    cleanup_pin(pins_left.pin_d);

    cleanup_pin(pins_right.pin_a);
    cleanup_pin(pins_right.pin_b);
    cleanup_pin(pins_right.pin_c);
    cleanup_pin(pins_right.pin_d);
}

// combined functions
void stepper::home_all(Horizontal &hor, Vertical &ver)
{
    std::thread tmp_thread(&stepper::Horizontal::move_absolute_angle, hor, 0);
    ver.move_absolute_angle(0);

    if (tmp_thread.joinable())
        tmp_thread.join();
}

void stepper::all_off(Horizontal &hor, Vertical &ver)
{
    hor.off();
    ver.off();
}

void stepper::all_shut(Horizontal &hor, Vertical &ver)
{
    hor.shutdown();
    ver.shutdown();
}
