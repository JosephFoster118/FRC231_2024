#include "SwerveModule.h"


void SwerveModule::setTargetAngle(double target)
{
    target_angle = target;
}

double SwerveModule::getTargetAngle()
{
    return target_angle;
}

void SwerveModule::setPower(double power_)
{
    power = power_;
}

double SwerveModule::getPower()
{
    return power;
}

