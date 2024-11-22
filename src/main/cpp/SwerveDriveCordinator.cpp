#include "SwerveDriveCordinator.h"

#include <cmath>
#include <iostream>

SwerveModuleCordinator::SwerveModuleCordinator(
    std::shared_ptr<SwerveModule> fl,
    std::shared_ptr<SwerveModule> fr,
    std::shared_ptr<SwerveModule> bl,
    std::shared_ptr<SwerveModule> br,
    double w_base,
    double t_width
):
front_left{fl},
front_right{fr},
back_left{bl},
back_right{br},
wheel_base{w_base},
track_width{t_width}
{
    //Calculate wheel angles from y+
    front_left_angle = Utility::toDegrees(std::atan2(track_width, wheel_base));
    front_right_angle = Utility::toDegrees(std::atan2(-track_width, wheel_base));
    back_left_angle = Utility::toDegrees(std::atan2(track_width, -wheel_base));
    back_right_angle = Utility::toDegrees(std::atan2(-track_width, -wheel_base));
}

void SwerveModuleCordinator::setDrive(double angle, double magnitude, double turn)
{
    using namespace Utility;

    if(field_oriented)
    {
        angle = get180Range(angle - field_angle);//TODO: May need to add
    }

    if(
        !front_left ||
        !front_right ||
        !back_left ||
        !back_right
    )
    {
        std::cout << "Not all motor pointers are set in the swerve cordinator!" << std::endl;
        return;
    }
    front_left->setPower(magnitude);
    front_right->setPower(magnitude);
    back_left->setPower(magnitude);
    back_right->setPower(magnitude);
    turn = std::min(1.0, std::max(-1.0, turn));
    double turn_angle = turn*45.0;
    if(std::abs(closestAngle(angle, front_left_angle)) >= 90.0)
    {
        front_left->setTargetAngle(angle + turn_angle);
    }
    else
    {
        front_left->setTargetAngle(angle - turn_angle);
    }
    if(std::abs(closestAngle(angle, front_right_angle)) >= 90.0)
    {
        front_right->setTargetAngle(angle + turn_angle);
    }
    else
    {
        front_right->setTargetAngle(angle - turn_angle);
    }
    if(std::abs(closestAngle(angle, back_left_angle)) >= 90.0)
    {
        back_left->setTargetAngle(angle + turn_angle);
    }
    else
    {
        back_left->setTargetAngle(angle - turn_angle);
    }
    if(std::abs(closestAngle(angle, back_right_angle)) >= 90.0)
    {
        back_right->setTargetAngle(angle + turn_angle);
    }
    else
    {
        back_right->setTargetAngle(angle - turn_angle);
    }
}

void SwerveModuleCordinator::setRotate(double turn)
{
    if(
        !front_left ||
        !front_right ||
        !back_left ||
        !back_right
    )
    {
        std::cout << "Not all motor pointers are set in the swerve cordinator!" << std::endl;
        return;
    }

    front_left->setPower(turn);
    front_right->setPower(turn);
    back_left->setPower(turn);
    back_right->setPower(turn);

    front_left->setTargetAngle(front_left_angle - 90);
    front_right->setTargetAngle(front_right_angle - 90);
    back_left->setTargetAngle(back_left_angle - 90);
    back_right->setTargetAngle(back_right_angle - 90);
}

void SwerveModuleCordinator::stopDrive()
{
    front_left->setPower(0.0);
    front_right->setPower(0.0);
    back_left->setPower(0.0);
    back_right->setPower(0.0);
}

void SwerveModuleCordinator::update(std::chrono::duration<double> seconds_passed)
{
    front_left->update(seconds_passed);
    front_right->update(seconds_passed);
    back_left->update(seconds_passed);
    back_right->update(seconds_passed);
}

void SwerveModuleCordinator::setFieldOriented(bool is_field_oriented)
{
    field_oriented = is_field_oriented;
}

bool SwerveModuleCordinator::isFieldOriented()
{
    return field_oriented;
}

void SwerveModuleCordinator::setFieldAngle(double angle)
{
    field_angle = angle;
}
