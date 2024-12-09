#include <iostream>
#include <map>
#include <fcntl.h>  // For file control options (open, O_RDWR, etc.)
#include <unistd.h> // For close()
#include <cstring>  // For strerror()

#include "serial.hpp"


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

bool serial::read_message(int port, std::string &buffer, int timeout)
{
// bool read_string(std::string &buffer, int serial_fd, unsigned int timeout) {
    buffer.clear(); // Ensure the buffer starts empty
    unsigned int current_timeout = 0;
    const unsigned int delay_interval = 5; // 5 ms delay between checks

    while (current_timeout < timeout) {
        char tmp;

        // Check if data is available on the serial port
        ssize_t bytes_read = read(port, &tmp, 1);
        if (bytes_read > 0) {
            current_timeout = 0; // Reset timeout when data is received

            if (tmp == '\0') {
                break; // Null character indicates end of string
            }

            buffer.push_back(tmp); // Append character to buffer
            continue;
        }

        // No data available, wait and increment timeout
        usleep(delay_interval);
        current_timeout += delay_interval;
    }

    return !buffer.empty(); // Return true if any data was read
}

int serial::close_port(int port)
{
    return close(port);
}


SimpleSerial::SimpleSerial(const char *device)
  : device(device)
{}

SimpleSerial::~SimpleSerial()
{
    if (port != -1)
    {
        close();
    }
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

int SimpleSerial::write(const std::string &message)
{
    return serial::write_message(port, message.c_str());
}

int SimpleSerial::write_json(json data)
{
    // dump to json and add \0 as terminator
    return write(data.dump() + '\0');
}

bool SimpleSerial::read(std::string &buffer, int timeout)
{
    return serial::read_message(port, buffer, timeout);
}

json SimpleSerial::read_json(int timeout)
{
    // read data as string
    std::string read_buff;
    // if (!read(read_buff))
        // return false;

    read(read_buff);

    // convert to json
    return json::parse(read_buff);
}

int SimpleSerial::close()
{
    return serial::close_port(port);
}
