#pragma once

#include <chrono>

#include "Utility.h"

#include <frc/kinematics/SwerveModulePosition.h>

class SwerveModule
{
public:

    virtual double getAngle() = 0;
    virtual void setMotors() = 0;
    double getTargetAngle();
    void setTargetAngle(double target);
    void setPower(double power_);
    double getPower();
    virtual void update(std::chrono::duration<double> seconds_passed) = 0;
    virtual frc::SwerveModulePosition getModulePosition() = 0;

protected:
    double target_angle{0};
    double power{0};

};

