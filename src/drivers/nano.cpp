#include <unistd.h>

#include "nano.hpp"


// message type values
#define TYPE_SEND           0
#define TYPE_RECEIVE        1
#define TYPE_LASER_MEASURE  2
#define TYPE_SET            3

// message (type 3) target values
#define LASER_STATE         0
#define LASER_RESOLUTION    1
#define LASER_RANGE         2
#define FAN_SPEED           3


using namespace nano;


Nano::Nano(const char *device)
  : SimpleSerial(device)
{}


bool Nano::begin(int baud)
{
    return SimpleSerial::begin(baud);
}

bool Nano::active()
{
    return SimpleSerial::active();
}

bool Nano::set_laser_range(uint8_t range)
{
    // send command to nano
    // clear_input();
    int message_id = write_json({
        {"type", TYPE_SET},
        {"target", LASER_RANGE},
        {"value", range}
    });

    // receive response
    auto [status, data] = try_receive_reply(message_id, 200000);

    if (!status)
        return false;

    return data["ack"];
}

bool Nano::set_laser_resolution(bool resolution)
{
    // send command to nano
    // clear_input();
    int message_id = write_json({
        {"type", TYPE_SET},
        {"target", LASER_RESOLUTION},
        {"value", resolution}
    });

    // receive response
    auto [status, data] = try_receive_reply(message_id, 200*MS);

    if (!status)
        return false;

    return data["ack"];
}

bool Nano::set_laser_state(bool state)
{
    // send command to nano
    // clear_input();
    int message_id = write_json({
        {"type", TYPE_SET},
        {"target", LASER_STATE},
        {"value", state}
    });

    // receive response
    auto [status, data] = try_receive_reply(message_id, 200000);

    if (debugging)
        std::cout << status << "=status, checking \""<< data <<"\" for valid status" << std::endl;

    // check for invalid data
    if (!status)
        return -1;

    return data["ack"];
}

double Nano::laser_measure()
{
    // send command to nano
    // clear_input();
    int message_id = write_json({
        {"type", TYPE_LASER_MEASURE}
    });

    // receive response
    auto [status, data] = try_receive_reply(message_id, 2*S);

    if (debugging)
        std::cout << status << "=status, checking \""<< data <<"\" for status" << std::endl;

    if (!status)
        return -1;

    // check for invalid data
    if (debugging)
        std::cout << status << "=status, checking \""<< data <<"\" for valid data" << std::endl;

    if (data["type"] != 2)
    {
        std::cout << "invalid response to measurement: " << data.dump(4) << std::endl;
        return -1;
    }

    if (!data["valid"])
        return -1;

    if (debugging)
        std::cout << "returning value" << std::endl;

    return data["distance"];
}

bool Nano::set_fan_speed(uint8_t speed)
{
    // send command to nano
    // clear_input();
    int message_id = write_json({
        {"type", TYPE_SET},
        {"target", FAN_SPEED},
        {"value", speed}
    });

    // receive response
    auto [status, data] = try_receive_reply(message_id, 200000);

    if (!status)
        return false;

    std::cout << data << std::endl;

    return data["ack"];
}
