#pragma once

#include <vector>
#include <optional>
#include <frc/DriverStation.h>

namespace Utility
{

static const inline double PI = 3.141592653589793238463;

double get180Range(double angle);
double closestAngle(double a, double b);
double toDegrees(double radians);
double toRadians(double degrees);

template<typename T>
T minMax(T min, T max, T value)
{
    if(value < min)
    {
        return min;
    }
    else if(value > max)
    {
        return max;
    }

    return value;
}

template<typename T>
T deadZone(T input, T deadzone)
{
    if(std::abs(input) <= deadzone)
    {
        return 0;
    }
    return input;
}


}

