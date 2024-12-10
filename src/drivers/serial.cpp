#include <iostream>
#include <map>
#include <fcntl.h>  // For file control options (open, O_RDWR, etc.)
#include <unistd.h> // For close()
#include <cstring>  // For strerror()

#include "serial.hpp"


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
        std::cerr << "Error getting terminal attributes: " << strerror(errno) << std::endl;
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
        std::cerr << "Error setting terminal attributes: " << strerror(errno) << std::endl;
        exit(EXIT_FAILURE);
    }
}

int serial::setup(const char *device, int baud)
{
    // Open the serial port
    int serialPort = open(device, O_RDWR | O_NOCTTY | O_SYNC);
    if (serialPort < 0)
    {
        std::cerr << "Error opening serial port: " << strerror(errno) << std::endl;
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
        std::cerr << "Error flushing serial port: " << strerror(errno) << std::endl;
    }
}

int serial::write_message(int port, const char *message)
{
    ssize_t bytesWritten = write(port, message, strlen(message));
    if (bytesWritten < 0)
    {
        std::cerr << "Error writing to serial port: " << strerror(errno) << std::endl;
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
        std::cout << "waiting to receive data" << std::endl;

    while (current_timeout < timeout)
    {
        char tmp;

        // check if data is available on the serial port
        ssize_t bytes_read = read_with_timeout(port, &tmp, 1, delay_interval);
        if (bytes_read > 0)
        {
            if (debugging)
                std::cout << "received: \"" << tmp << "\"" << std::endl;

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
            std::cout << "\rno data available, timeout: " << current_timeout << std::endl;

        // No data available, wait and increment timeout
        // usleep(delay_interval);
        current_timeout += delay_interval;
    }

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
        std::cout << "receive thread started" << std::endl;
    
    while (running)
    {
        if (available())
        {
            if (debugging)
                std::cout << "message available" << std::endl;

            auto [ok, data] = read_json();

            if (debugging)
                std::cout << "received (ok=" << ok << "): \"" << data << "\"" << std::endl;

            if (!ok)
                continue;

            // check if message has an "reply to" key
            if (data.contains("to"))
            {
                if (debugging)
                    std::cout << "appending" << std::endl;

                received_messages.push_back(data);
            }
            else
            {
                if (debugging)
                    std::cout << "doesn't contain \"to\" key" << std::endl;
            }

            continue;
        }

        usleep(receive_delay);
    }

    if (debugging)
        std::cout << "receive thread exit" << std::endl;
}

std::pair<bool, json> SimpleSerial::try_get_reply(uint16_t id)
{
    // if (debugging)
    //     std::cout << "trying to find message with id: " << id << std::endl;

    // checks if a message with the correct reply id has been appended
    for (json message : received_messages)
    {
        if (message["to"] == id)
        {
            if (debugging)
                std::cout << "found: " << message << std::endl;

            return {true, message};
        }
    }

    // not found
    return {false, {{"ack", false}}};
}

std::pair<bool, json> SimpleSerial::try_receive_reply(uint16_t id, int timeout)
{
    if (debugging)
        std::cout << "trying to receive message with id: " << id << std::endl;
    
    while (timeout > 0)
    {
        auto [found, data] = try_get_reply(id);

        // return message if received
        if (found)
            return {true, data};
        
        usleep(receive_delay);
        timeout -= receive_delay;
    }

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
        std::cerr << "Unsupported baud rate: " << baud << std::endl;
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

// void SimpleSerial::clear_input()
// {
//     std::string buff;

//     if (debugging)
//         std::cout << "clearing input ..." << std::endl;

//     // read while available
//     while (available())
//     {
//         read(buff, 100);

//         if (debugging && buff.length() > 0)
//             std::cout << "cleaning read: " << buff << std::endl;
//     }
//     if (!available() && debugging)
//         std::cout << "clearing: nothing available" << std::endl;
// }

void SimpleSerial::flush()
{
    if (debugging)
        std::cout << "flushing ..." << std::endl;

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
        std::cout << "writing json: " << (data.dump() + '\0') << std::endl;

    // write data to serial
    write(data.dump() + '\0');

    // dump to json and add \0 as terminator
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
        std::cout << "waiting to receive" << std::endl;

    read(read_buff);

    if (read_buff.length() < 2)
    {
        if (debugging)
            std::cout << "invalid receive: \"" << read_buff << "\"" << std::endl;
        
        if (!available())
        {
            if (debugging)
                std::cout << "new message available, retrying" << std::endl;

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
        std::cout << "converting \"" << read_buff << "\" to json: ";
        for (unsigned char c : read_buff) {
            std::cout << "\\x" << std::hex << (int)c;
        }
        std::cout << std::endl;

        std::string sanitized = "{\"type\":2,\"valid\":false}";
    }

    // convert to json
    try
    {
        return {true, json::parse(read_buff)};
    }
    catch (const json::exception& e)
    {
        std::cerr << "JSON error: " << e.what() << std::endl;
        return {false, {{"type", 0}, {"ack", 0}, {"valid", 0}}};
    }
}

int SimpleSerial::close()
{
    int rtn = serial::close_port(port);
    port = -1;
    return rtn;
}
