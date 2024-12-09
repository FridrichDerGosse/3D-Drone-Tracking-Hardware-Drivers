/**
 * @file serial.hpp
 * @author Nilusink
 * @brief simple serial driver for communication with an arduino nano
 * @date 2024-12-09
 * 
 * @copyright Copyright Nilusink (c) 2024
 * 
 */
#pragma once
#include <termios.h>    // For POSIX terminal control definitions
#include <iostream>
#include <nlohmann/json.hpp>


using json = nlohmann::json;


namespace serial
{
    speed_t get_baud_rate(int baud);

    void configureSerialPort(int fd, int baudRate);

    class SimpleSerial
    {
        private:
            int port = -1;
            int baud_rate = -1;
            const char *device;
        
        public:
            SimpleSerial(const char *device);
            ~SimpleSerial();

            /**
             * @brief start the serial connection
             * 
             * @param baud 
             * @return int 
             */
            bool begin(int baud);

            /**
             * @brief checks if data is avaialable to read
             */
            bool available();

            /**
             * @brief send a message to a serial device
             * 
             * @param message message to send
             * @return int message length (negative for fail)
             */
            int write(const std::string &message);

            int write_json(json data);

            /**
             * @brief read message from serial device
             * 
             * @param buffer write target
             * @param timeout max time to wait
             * @return true successfully received
             * @return false no data
             */
            bool read(std::string &buffer, int timeout=50);

            json read_json(int timeout=50);

            /**
             * @brief close the serial connection
             */
            int close();
    };

    int setup(const char *device, int baud);

    int write_message(int port, const char *message);

    bool read_message(int port, std::string &buffer, int timeout=50);

    int close_port(int port);
}
