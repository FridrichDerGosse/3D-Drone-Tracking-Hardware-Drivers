#include <iostream>
#include <map>
#include <fcntl.h>  // For file control options (open, O_RDWR, etc.)
#include <unistd.h> // For close()
#include <cstring>  // For strerror()

#include "serial.hpp"
#include "terminal.hpp"


// chat-gpt function
ssize_t read_with_timeout(int fd, void* buf, size_t count, int timeout_ms) {
    fd_set read_fds;
    struct timeval timeout;

    // Configure the timeout
    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;

    // Initialize the set of file descriptors
    FD_ZERO(&read_fds);
    FD_SET(fd, &read_fds);

    // Wait for the file descriptor to become ready
    int result = select(fd + 1, &read_fds, nullptr, nullptr, &timeout);
    if (result < 0) {
        // Error occurred
        perror("select");
        return -1;
    } else if (result == 0) {
        // Timeout
        return 0; // No data available within the timeout
    }

    // File descriptor is ready; perform the read
    return read(fd, buf, count);
}

using namespace serial;

speed_t serial::get_baud_rate(int baud) {
    // Map of baud rates to their constants
    static const std::map<int, speed_t> baud_map = {
        {50, B50},     {75, B75},     {110, B110},   {134, B134},
        {150, B150},   {200, B200},   {300, B300},   {600, B600},
        {1200, B1200}, {1800, B1800}, {2400, B2400}, {4800, B4800},
        {9600, B9600}, {19200, B19200}, {38400, B38400}
    };

    // Find the baud rate in the map
    auto it = baud_map.find(baud);
    if (it != baud_map.end()) {
        return it->second; // Return the corresponding speed_t
    }

    return -1; // Return empty optional if baud rate not found
}

void serial::configureSerialPort(int fd, int baudRate)
{
    struct termios tty;

    // Get current configuration
    if (tcgetattr(fd, &tty) != 0)
    {
        std::cerr << "Error getting terminal attributes: " << strerror(errno) << cc::endl;
        exit(EXIT_FAILURE);
    }

    // Set input and output baud rates
    cfsetospeed(&tty, baudRate);
    cfsetispeed(&tty, baudRate);

    // Configure terminal options
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit characters
    tty.c_iflag &= ~IGNBRK;                     // Disable break processing
    tty.c_lflag = 0;                            // No signaling chars, no echo, no canonical processing
    tty.c_oflag = 0;                            // No remapping, no delays
    tty.c_cc[VMIN] = 1;                         // Read at least 1 character
    tty.c_cc[VTIME] = 1;                        // 0.1 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable XON/XOFF flow control
    tty.c_cflag |= (CLOCAL | CREAD);        // Enable the receiver and set local mode
    tty.c_cflag &= ~(PARENB | PARODD);      // No parity
    tty.c_cflag &= ~CSTOPB;                 // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;                // No hardware flow control

    // Apply the configuration
    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        std::cerr << "Error setting terminal attributes: " << strerror(errno) << cc::endl;
        exit(EXIT_FAILURE);
    }
}

int serial::setup(const char *device, int baud)
{
    // Open the serial port
    int serialPort = open(device, O_RDWR | O_NOCTTY | O_SYNC);
    if (serialPort < 0)
    {
        std::cerr << "Error opening serial port: " << strerror(errno) << cc::endl;
        return -1;
    }

    // Configure the serial port
    configureSerialPort(serialPort, baud); // Use 9600 baud rate
    return serialPort;
}

void serial::flush(int fd)
{
    // Clear input and output buffers
    if (tcflush(fd, TCIOFLUSH) != 0)
    {
        std::cerr << "Error flushing serial port: " << strerror(errno) << cc::endl;
    }
}

int serial::write_message(int port, const char *message)
{
    ssize_t bytesWritten = write(port, message, strlen(message));
    if (bytesWritten < 0)
    {
        std::cerr << "Error writing to serial port: " << strerror(errno) << cc::endl;
        close(port);
        return -1;
    }
    return bytesWritten;
}

bool serial::read_message(int port, std::string &buffer, int timeout, bool debugging)
{
    buffer.clear();
    unsigned int current_timeout = 0;
    const unsigned int delay_interval = 5; // 5 ms delay between checks

    if (debugging)
        std::cout << FG_COLOR(250) << "waiting to receive data" << cc::endl;

    while (current_timeout < timeout)
    {
        char tmp;

        // check if data is available on the serial port
        ssize_t bytes_read = read_with_timeout(port, &tmp, 1, delay_interval);
        if (bytes_read > 0)
        {
            if (debugging)
                std::cout << FG_COLOR(22) << "received" << FG_COLOR(240) << ": \"" << tmp << "\"" << cc::endl;

            current_timeout = 0;

            // terminator
            if (tmp == '\0')
            {
                break;
            }

            buffer.push_back(tmp); // Append character to buffer
            continue;
        }

        if (debugging)
            std::cout << FG_COLOR(250) << "\rno data available, timeout: " << current_timeout << cc::endl;

        // No data available, wait and increment timeout
        current_timeout += delay_interval;
    }

    if (debugging && current_timeout >= timeout)
        std::cout << FG_COLOR(52) << "\rtimeout" << cc::endl;

    return !buffer.empty(); // Return true if any data was read
}

int serial::close_port(int port)
{
    return close(port);
}

int SimpleSerial::get_unique_id()
{
    return id_counter.fetch_add(1);
}

void SimpleSerial::receive_messages()
{
    if (debugging)
        std::cout << cc::bfg::CYAN << "receive thread started" << cc::endl;
    
    while (running)
    {
        if (available())
        {
            if (debugging)
                std::cout << FG_COLOR(250) << "message available" << cc::endl;

            auto [ok, data] = read_json();

            if (debugging)
            {
                std::cout << FG_COLOR(250) << "received (";
                std::cout << (ok ? cc::fg::GREEN : cc::fg::RED) << (ok ? "OK" : "FAIL");
                std::cout << FG_COLOR(250) << "): \"" << data << "\"" << cc::endl;
            }

            if (!ok)
                continue;

            // check if message is a control message
            if (data.contains("ctrl"))
            {
                switch ((int)data["ctrl"])
                {
                    case 0:  // comm start
                    {
                        if (debugging)
                            std::cout << FG_COLOR(250) << "received comm start" << cc::endl;

                        connection_active = true;
                        break;
                    }
                    
                    case 1: // ping request
                    {
                        if (debugging)
                            std::cout << FG_COLOR(250) << "received ping request" << cc::endl;
                        
                        // send pong message
                        write_json({
                            {"type", 2},
                            {"to", data["id"]}
                        });

                        break;
                    }

                    case 2:  // pong
                    {
                        if (debugging)
                            std::cout << FG_COLOR(250) << "received pong with id " << FG_COLOR(240) << data["to"] << cc::endl;

                        // append to message buffer
                        received_replies.push_back(data);

                        break;
                    }

                    case 3:  // rf message forward
                    {
                        if (net_debugging)
                            std::cout << cc::fg::BLUE << "received RF message: " << FG_COLOR(245) << data["data"].dump(4) << cc::endl;
                        
                        // append data to network messages
                        // differentiate between replies and normal messages
                        if (data["data"].contains("to"))
                        {
                            if (net_debugging)
                                std::cout << FG_COLOR(250) << "appending as reply" << cc::endl;

                            received_network_replies.push_back(data["data"]);
                        }
                        else
                        {
                            if (net_debugging)
                                std::cout << FG_COLOR(250) << "appending as message" << cc::endl;

                            received_network_messages.push_back(data["data"]);
                        }

                        break;
                    }

                    case 4:  // nano debugging message
                    {
                        // save as string so no extra > " < are added
                        std::string content = data["content"];
                        std::cout << cc::fg::CYAN << "nano debug>> " << cc::ctrl::ENDC << content << cc::endl;
                        break;
                    }

                    default:
                    {
                        if (debugging)
                            std::cout << cc::fg::YELLOW << "invalid control message: " << data.dump(4) << cc::endl;

                        break;
                    }
                }
            }

            // check if message has an "reply to" key
            else if (data.contains("to"))
            {
                if (debugging)
                    std::cout << "appending" << cc::endl;

                received_replies.push_back(data);
            }
            else
            {
                if (debugging)
                    std::cout << "doesn't contain \"to\" key" << cc::endl;
            }

            continue;
        }

        usleep(receive_delay);
    }

    if (debugging)
        std::cout << "receive thread exit" << cc::endl;
}

std::pair<bool, json> SimpleSerial::try_get_from_list(std::list<json> *list, uint16_t id)
{
    // checks if a message with the correct reply id has been appended
    for (json message : *list)
    {
        if (message["to"] == id)
        {
            if (debugging)
                std::cout << FG_COLOR(22) << "found: " << FG_COLOR(250) << message << cc::endl;

            // delete element from list
            list->remove(message);

            // return message
            return {true, message};
        }
    }

    // not found
    return {false, {{"ack", false}}};
}

std::pair<bool, json> SimpleSerial::try_get_reply(uint16_t id)
{
    return try_get_from_list(&received_replies, id);
}

std::pair<bool, json> SimpleSerial::try_get_netowrk_reply(uint16_t id)
{
    return try_get_from_list(&received_network_replies, id);
}

std::pair<bool, json> SimpleSerial::try_receive_reply(
    uint16_t id,
    int timeout,
    bool type
)
{
    if (debugging)
        std::cout << FG_COLOR(250) << "trying to receive message with id: " << cc::fg::MAGENTA << id << cc::endl;
    
    while (timeout > 0)
    {
        // select list depending on type
        auto [found, data] = try_get_from_list(
            &(type ? received_network_replies : received_replies),
            id
        );

        // return message if received
        if (found)
            return {true, data};
        
        usleep(receive_delay);
        timeout -= receive_delay;
    }

    if (debugging)
        std::cout << FG_COLOR(52) << "failed to receive message (id=" << cc::fg::MAGENTA << id << ")" << cc::endl;

    // return dummy false reply
    return {false, {{"type", 0}, {"ack", 0}, {"valid", 0}}};
}

SimpleSerial::SimpleSerial(const char *device)
  : device(device)
{}

SimpleSerial::~SimpleSerial()
{
    // tell thread to stop running
    running = false;

    // close serial port if it hasn't already been done
    if (port != -1)
    {
        close();
    }

    // wait for thread to exit
    if (receive_thread.joinable())
        receive_thread.join();
}

bool SimpleSerial::begin(int baud)
{
    baud_rate = baud;

    // convert number baud rate  (9600, ...) to their corresponding values
    speed_t actual_baud = serial::get_baud_rate(baud);

    if (actual_baud < 0)
    {
        std::cerr << "Unsupported baud rate: " << baud << cc::endl;
        return false;
    }

    port = serial::setup(device, baud);

    // start receive function
    receive_thread = std::thread(&SimpleSerial::receive_messages, this);

    return true;
}

bool SimpleSerial::available()
{
    fd_set read_fds;
    struct timeval timeout;

    // Clear the file descriptor set
    FD_ZERO(&read_fds);
    FD_SET(port, &read_fds);

    // Set the timeout (0 for no blocking)
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;

    // Check if data is available
    int result = select(port + 1, &read_fds, NULL, NULL, &timeout);
    return result > 0; // If result is greater than 0, data is available
}

bool SimpleSerial::active()
{
    return connection_active;
}

// void SimpleSerial::clear_input()
// {
//     std::string buff;

//     if (debugging)
//         std::cout << "clearing input ..." << cc::endl;

//     // read while available
//     while (available())
//     {
//         read(buff, 100);

//         if (debugging && buff.length() > 0)
//             std::cout << "cleaning read: " << buff << cc::endl;
//     }
//     if (!available() && debugging)
//         std::cout << "clearing: nothing available" << cc::endl;
// }

void SimpleSerial::flush()
{
    if (debugging)
        std::cout << FG_COLOR(117) << "flushing ..." << cc::endl;

    serial::flush(port);
}

int SimpleSerial::write(const std::string &message)
{
    // write message to serial
    int rcode = serial::write_message(port, message.c_str());

    // make sure message was written
    flush();

    return rcode;
}

int SimpleSerial::write_json(json data)
{
    // tag message with unique id
    data["id"] = get_unique_id();

    if (debugging)
        std::cout << FG_COLOR(250) << "writing json: " << FG_COLOR(240) << data.dump(4) << cc::endl;

    // write data to serial
    write(data.dump() + '\0');

    // dump to json and add \0 as terminator
    return data["id"];
}

int SimpleSerial::write_json_network(json data, int target_node)
{
    // tag network message with unique id
    data["id"] = get_unique_id();

    if (debugging)
        std::cout << FG_COLOR(250) << "encapsupating network message" << FG_COLOR(240) << data.dump(4) << cc::endl;

    // encapsulate into nano message
    json encapsulated_message;
    encapsulated_message["type"] = 0;
    encapsulated_message["target"] = target_node;
    encapsulated_message["data"] = data;

    // send message and wait for acknowledgement
    int nano_id = write_json(encapsulated_message);
    auto [ok, reply_data] = try_receive_reply(nano_id, 100*MS);

    // return fail if ack wasn't received or isn't successful
    if (!ok)
        return -1;

    if (!reply_data["ack"])
        return -1;

    // if succesfull, return network message id
    return data["id"];
}

bool SimpleSerial::read(std::string &buffer, int timeout)
{
    return serial::read_message(port, buffer, timeout);
}

std::pair<bool, json> SimpleSerial::read_json(int timeout)
{
    // read data as string
    std::string read_buff;
    // if (!read(read_buff))
        // return false;

    if (debugging)
        std::cout << FG_COLOR(250) << "waiting to receive" << cc::endl;

    read(read_buff);

    if (read_buff.length() < 2)
    {
        if (debugging)
            std::cout << cc::fg::RED << "invalid receive: " << FG_COLOR(240) << read_buff << cc::endl;
        
        if (!available())
        {
            if (debugging)
                std::cout << FG_COLOR(250) << "new message available, retrying" << cc::endl;

            return read_json(timeout);
        }

        // create a fake false acknowledgement for receive fail
        return {false, {{"type", 0}, {"ack", 0}, {"valid", 0}}};
    }

    // clean string up
    // read_buff.erase(std::remove_if(read_buff.begin(), read_buff.end(), ::isspace), read_buff.end());
    read_buff.erase(std::remove(read_buff.begin(), read_buff.end(), '\0'), read_buff.end());


    if (debugging)
    {
        std::cout << FG_COLOR(250) << "converting " << FG_COLOR(240) << read_buff << FG_COLOR(250) << " to json: " << FG_COLOR(240);
        for (unsigned char c : read_buff) {
            std::cout << "\\x" << std::hex << (int)c;
        }
        std::cout << cc::endl;

        std::string sanitized = "{\"type\":2,\"valid\":false}";
    }

    // convert to json
    try
    {
        return {true, json::parse(read_buff)};
    }
    catch (const json::exception& e)
    {
        std::cerr << "JSON error: " << e.what() << cc::endl;
        return {false, {{"type", 0}, {"ack", 0}, {"valid", 0}}};
    }
}

int SimpleSerial::close()
{
    int rtn = serial::close_port(port);
    port = -1;
    return rtn;
}
