#include "PID.h"

PID::PID(double p_constant, double i_constant, double d_constant, double e_c, double p_exp)
{
    p_c = p_constant;
    i_c = i_constant;
    d_c = d_constant;
    error_cutoff = e_c;
}
PID::PID(const PID& old)
{
    p_c = old.p_c;
    i_c = old.i_c;
    d_c = old.d_c;
    error_cutoff = old.error_cutoff;
    p_e = old.p_e;
}

double PID::calculatePID(double current, double target)
{
    float p = target - current;
    float d{0};
    TimePoint time_point_now = std::chrono::high_resolution_clock::now();
    if(last_time != TimePoint{})
    {
        auto delta_time = std::chrono::duration_cast<std::chrono::milliseconds>(time_point_now - last_time).count();
        if(error_cutoff == 0.0 || std::abs(p) <= error_cutoff)
        {
            error += p*delta_time;
        }
        else
        {
            error = 0.0;
        }// If we are far distance away -> Don't count error -> Set 0
        // 
        
        
        if(last_error)
        {
            d = (p - last_error)/delta_time;
        }
        else
        {
            d = 0;
        }
        
        
        last_error = p;
    }

    last_time = time_point_now;
    float result = std::pow(p*p_c, p_e) + error*i_c - d*d_c;
    return result;
}

double PID::calculatePID(double current, double target, std::chrono::duration<double> seconds_passed)
{
    float p = target - current;
    float d{0};
    
    auto delta_time = std::chrono::duration_cast<std::chrono::milliseconds>(seconds_passed).count();
    if(error_cutoff == 0.0 || std::abs(p) <= error_cutoff)
    {
        error += p*delta_time;
    }
    else
    {
        error = 0.0;
    }// If we are far distance away -> Don't count error -> Set 0
    // 
    
    
    if(last_error)
    {
        d = (p - last_error)/delta_time;
    }
    else
    {
        d = 0;
    }
    
    last_error = p;

    last_time = std::chrono::high_resolution_clock::now();
    float result = std::pow(p*p_c, p_e) + error*i_c - d*d_c;
    return result;
}

void PID::resetError()
{
    last_time = TimePoint{};
    last_error = 0;
    error = 0;
}

double PID::getErrror()
{
    return error;
}
