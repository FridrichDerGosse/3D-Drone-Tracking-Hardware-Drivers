#include "helper.hpp"
#include <iostream>


double map(double value, double in_min, double in_max, double out_min, double out_max)
{
    return (value - in_min) * (out_max - out_min) / in_max + out_min;
}
