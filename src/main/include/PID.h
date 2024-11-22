#pragma once
#include <chrono>
#include <cmath>


class PID
{
public:
    PID() = delete;
    PID(const PID& old);
    PID(double p_constant, double i_constant, double d_constant, double e_c = 0.0, double p_exp = 1.0);

    double calculatePID(double current, double target);
    double calculatePID(double current, double target, std::chrono::duration<double> seconds_passed);
    void resetError();
    double getErrror();

private:
    using TimePoint = std::chrono::time_point<std::chrono::high_resolution_clock>;
    double p_c{0.0};
    double i_c{0.0};
    double d_c{0.0};
    double error{0.0};
    double last_error{0};
    double error_cutoff{0.0};
    double p_e{1.0};


    TimePoint last_time{};
    
};


