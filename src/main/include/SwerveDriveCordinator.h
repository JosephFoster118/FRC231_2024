#pragma once

#include <memory>

#include "SwerveModule.h"


class SwerveModuleCordinator
{
public:
    
    SwerveModuleCordinator() = delete;
    SwerveModuleCordinator(const SwerveModuleCordinator&) = delete;

    SwerveModuleCordinator(
        std::shared_ptr<SwerveModule> fl,
        std::shared_ptr<SwerveModule> fr,
        std::shared_ptr<SwerveModule> bl,
        std::shared_ptr<SwerveModule> br,
        double w_base,
        double t_width
    );

    void setDrive(double angle, double magnitude, double turn);
    void setRotate(double turn);
    void stopDrive();
    void update(std::chrono::duration<double> seconds_passed = std::chrono::milliseconds{50});

    void setFieldOriented(bool is_field_oriented);
    bool isFieldOriented();
    void setFieldAngle(double angle);

private:

    std::shared_ptr<SwerveModule> front_left{nullptr};
    std::shared_ptr<SwerveModule> front_right{nullptr};
    std::shared_ptr<SwerveModule> back_left{nullptr};
    std::shared_ptr<SwerveModule> back_right{nullptr};

    double wheel_base;
    double track_width;

    double front_left_angle{0};
    double front_right_angle{0};
    double back_left_angle{0};
    double back_right_angle{0};

    bool field_oriented{true};
    double field_angle{0.0};

};

