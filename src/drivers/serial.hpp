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
#include <list>
#include <atomic>
#include <thread>
#include <termios.h>    // For POSIX terminal control definitions
#include <iostream>
#include <nlohmann/json.hpp>
#include "helper.hpp"


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
            bool connection_active = false;

            const char *device;
            const uint16_t receive_delay = 5 * MS;

            std::atomic_bool running{true};
            std::atomic_uint32_t id_counter{0};

            std::list<json> received_network_messages;
            std::list<json> received_network_replies;
            std::list<json> received_replies;
            std::thread receive_thread;
       
        protected:
            /**
             * @brief Get the unique id objectget unique message
             */
            int get_unique_id();

            /**
             * @brief continually receive messages
             */
            void receive_messages();

            std::pair<bool, json> try_get_from_list(std::list<json> *list, uint16_t id);

            /**
             * @brief try to find a reply message
             * 
             * @param id original message id
             * @return found, json (if found) 
             */
            std::pair<bool, json> try_get_reply(uint16_t id);

            /**
             * @brief try to find a network reply message
             * 
             * @param id original message id
             * @return found, json (if found) 
             */
            std::pair<bool, json> try_get_netowrk_reply(uint16_t id);

            /**
             * @brief tries to receive a reply from either nano or network
             * 
             * @param id id to find
             * @param timeout max time to wait
             * @param type 0: nano, 1: network
             * @return found, json (if found)
             */
            std::pair<bool, json> try_receive_reply(
                uint16_t id,
                int timeout,
                bool type=false
            );

        public:
            bool net_debugging = false;
            bool debugging = false;

            SimpleSerial(const char *device);
            ~SimpleSerial();

            /**
             * @brief start the serial connection
             * 
             * @param baud baudrate
             * @return true successfully begun
             * @return false invalid baud rate
             */
            bool begin(int baud);

            /**
             * @brief checks if data is avaialable to read
             */
            bool available();

            /**
             * @brief checks if the connection with the nano is active (nano has send ctrl type 0 message)
             */
            bool active();

            /**
             * @brief if available, read all input
             */
            // void clear_input();

            /**
             * @brief flush all messages to serial
             */
            void flush();

            /**
             * @brief send a message to a serial device
             * 
             * @param message message to send
             * @return int message length (negative for fail)
             */
            int write(const std::string &message);

            /**
             * @brief send a json object over serial
             * 
             * @param data json object
             * @return int message id (negative for fail)
             */
            int write_json(json data);

            /**
             * @brief send a json object over network
             * 
             * @param data json object
             * @param target_node network target
             * @return int message id (negative for fail)
             */
            int write_json_network(json data, int target_node);

            /**
             * @brief read message from serial device
             * 
             * @param buffer write target
             * @param timeout max time to wait
             * @return true successfully received
             * @return false no data
             */
            bool read(std::string &buffer, int timeout=50);

            /**
             * @brief read a json object from serial
             * 
             * @param timeout max time to wait
             * @return json json data
             */
            std::pair<bool, json> read_json(int timeout=50);

            /**
             * @brief close the serial connection
             */
            int close();
    };

    int setup(const char *device, int baud);

    void flush(int fd);

    int write_message(int port, const char *message);

    bool read_message(int port, std::string &buffer, int timeout=50, bool debugging=false);

    int close_port(int port);
}
