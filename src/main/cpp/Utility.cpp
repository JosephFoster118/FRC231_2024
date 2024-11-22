#include "Utility.h"

#include <cmath>

namespace Utility
{


double get180Range(double angle)
{
    angle = std::fmod(std::fmod(angle, 360) + 360, 360);
    if(angle > 180)
    {
        angle -= 360;
    }
    return angle;
}

double closestAngle(double a, double b)
{
    double a_mod = std::fmod(a, 360);
    double b_mod = std::fmod(b, 360);
    return get180Range(b_mod - a_mod);
}

double toDegrees(double radians)
{
    return radians * (180.0/PI);
}

double toRadians(double degrees)
{
    return degrees * (PI/180.0);
}


}

