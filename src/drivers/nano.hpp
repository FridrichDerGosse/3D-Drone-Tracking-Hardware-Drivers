/**
 * @file nano.hpp
 * @author Nilusink
 * @brief wrappers for json comms
 * @date 2024-12-09
 * 
 * @copyright Copyright Nilusink (c) 2024
 * 
 */
#include "serial.hpp"


namespace nano
{
    class Nano : protected serial::SimpleSerial
    {
        private:
            unsigned int timeout = 500;

        public:
            Nano(const char *device);

            /**
             * @brief start the serial connection
             * 
             * @param baud baudrate
             * @return true successfully begun
             * @return false invalid baud rate
             */
            bool begin(int baud);

            /**
             * @brief checks if the connection with the nano is active (nano has send ctrl type 0 message)
             */
            bool active();

            /**
             * @brief sensor range
             * 
             * @param range must be 5, 10, 30, 50 or 80
             * @return true success
             * @return false fail
             */
            bool set_laser_range(uint8_t range);

            /**
             * @brief set the sensor resolution
             * 
             * @param resolution 0: 1mm, 1: 0.1mm
             * @return true success
             * @return false fail
             */
            bool set_laser_resolution(bool resolution);

            /**
             * @brief turn laser on / off
             * 
             * @param state 0: off, 1: on
             * @return true success
             * @return false fail
             */
            bool set_laser_state(bool state);

            /**
             * @brief measure distances with the laser
             * 
             * @return double distance in m, negative values are fail
             */
            double laser_measure();

            /**
             * @brief change fan speed
             * 
             * @param speed 0-255
             */
            bool set_fan_speed(uint8_t speed);
    };
}