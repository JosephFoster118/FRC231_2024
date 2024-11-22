#include "MK4SwerveModule.h"
#include "Utility.h"

#include <cmath>
#include <iostream> //For debugging
#include <frc/Timer.h>

#include <frc/smartdashboard/SmartDashboard.h>

MK4SwerveModule::MK4SwerveModule(uint8_t drive_id, uint8_t rotate_id, uint8_t encoder_id, double encoder_offset, std::string module_name)
{
    drive_motor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(drive_id);
    rotate_motor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(rotate_id);
    rotate_motor->SetInverted(false);
    drive_motor->SetInverted(false);
    rotate_motor->SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
    drive_motor->SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
    encoder = std::make_unique<ctre::phoenix6::hardware::CANcoder>(encoder_id);
    offset = encoder_offset + ALL_ENCODER_OFFSET;
    name = module_name;
    if(name != "NA")
    {
        frc::SmartDashboard::SetDefaultNumber("SwerveOffset" + name, 0.0);
        frc::SmartDashboard::SetDefaultNumber("FeedForward" + name, 0.0);
        frc::SmartDashboard::SetDefaultNumber("Max" + name, 0.0);
        frc::SmartDashboard::SetDefaultNumber("rotate_temp" + name, rotate_motor->GetDeviceTemp().GetValue().value());
        frc::SmartDashboard::SetDefaultNumber("drive_temp" + name, drive_motor->GetDeviceTemp().GetValue().value());
    }
}

double MK4SwerveModule::getAngle()
{
    //return Utility::get180Range(((rotate_motor->GetSelectedSensorPosition()/2048)*360)/12.8);
    return Utility::get180Range((encoder->GetAbsolutePosition().GetValue().value())*360.0 - offset);
}

double MK4SwerveModule::getRawAngle()
{
    return Utility::get180Range(encoder->GetAbsolutePosition().GetValue().value()*360.0);
}

double MK4SwerveModule::getEncoderDistance()
{
    return drive_motor->GetPosition().GetValue().value();
}

void MK4SwerveModule::setMotors()
{

}

void MK4SwerveModule::setDriveVoltage(units::volt_t volts)
{
    drive_motor->SetVoltage(volts);
}

void MK4SwerveModule::setRotateVoltage(units::volt_t volts)
{
    rotate_motor->SetVoltage(volts);
}

units::volt_t MK4SwerveModule::getDriveVoltage()
{
    return static_cast<units::volt_t>(drive_motor->GetMotorVoltage().GetValueAsDouble());
}

units::meter_t MK4SwerveModule::getDrivePosition()
{
    return units::meter_t(drive_motor->GetPosition().GetValueAsDouble());
}

units::meters_per_second_t MK4SwerveModule::getDriveVelocity()
{
    return units::meters_per_second_t(drive_motor->GetVelocity().GetValueAsDouble());
}

units::volt_t MK4SwerveModule::getRotateVoltage()
{
    return static_cast<units::volt_t>(rotate_motor->GetMotorVoltage().GetValueAsDouble());
}

units::meter_t MK4SwerveModule::getRotatePosition()
{
    return units::meter_t((encoder->GetAbsolutePosition().GetValue().value())*360.0);
}

units::meters_per_second_t MK4SwerveModule::getRotateVelocity()
{
    return units::meters_per_second_t(encoder->GetVelocity().GetValueAsDouble()*360.0);
}

void MK4SwerveModule::update(std::chrono::duration<double> seconds_passed)
{
    if(!angle_pid || !angle_feedforward || !drive_feedforward || !drive_pid)
    {
        std::cout << "angle_pid never initialized!" << std::endl;
        rotate_motor->SetVoltage(units::voltage::volt_t{0});
        drive_motor->SetVoltage(units::voltage::volt_t{0});
        return;
    }
    using namespace Utility;
    using namespace units;
    double current_angle = getAngle();
    double set_angle = closestAngle(current_angle, target_angle);
    double set_angle_flipped = closestAngle(current_angle, target_angle + 180.0);
    double new_target = 0;
    bool is_forward = false;
    if(std::abs(set_angle) <= std::abs(set_angle_flipped))
    {
        new_target = current_angle + set_angle;
        is_forward = true;
    }
    else
    {
        new_target = current_angle + set_angle_flipped;
        is_forward = false;
    }

    //Turn motor
    double degrees_off = Utility::get180Range(current_angle - new_target);
    if(std::abs(degrees_off) > 1.5)
    {
        angle_pid->SetGoal(units::meter_t{0.0});
        double turn_power = angle_pid->Calculate(units::meter_t(degrees_off));
        auto turn_feed = angle_feedforward->Calculate(unit_t<compound_unit<meters, inverse<seconds>>>(degrees_off * 8));
        
        rotate_motor->SetVoltage(units::voltage::volt_t{turn_power} - turn_feed);
        if(name != "NA")
        {
            frc::SmartDashboard::PutNumber("SwerveOffset" + name, degrees_off);
            frc::SmartDashboard::PutNumber("FeedForward" + name, turn_feed.value());
            frc::SmartDashboard::PutNumber("Max" + name, drive_speed_max);
        }
    }
    else
    {
        rotate_motor->StopMotor();
    }

    //Drive motor
    double new_drive_power = power*max_drive_speed;
    if(!is_forward)
    {
        new_drive_power = -new_drive_power;
    }
    auto drive_pid_value = units::volt_t{drive_pid->Calculate(drive_motor->GetVelocity().GetValue().value(), new_drive_power)};
    auto drive_ff_value = drive_feedforward->Calculate(units::meters_per_second_t{new_drive_power});
    new_drive_power = new_drive_power * std::cos(Utility::toRadians(degrees_off));
    drive_motor->SetVoltage(drive_pid_value + drive_ff_value);

    drive_speed_max = std::max(drive_speed_max, drive_motor->GetVelocity().GetValue().value());
    if(name != "NA")
    {
        frc::SmartDashboard::PutNumber("rotate_temp" + name, rotate_motor->GetDeviceTemp().GetValue().value());
        frc::SmartDashboard::PutNumber("drive_temp" + name, drive_motor->GetDeviceTemp().GetValue().value());
    }
}

void MK4SwerveModule::setAnglePID(const PIDConfig& pid, const TrapazoidalProfileConfig& trap)
{
    using namespace units;
    auto trap_const = frc::TrapezoidProfile<units::meters>::Constraints(
        unit_t<compound_unit<meters, inverse<seconds>>>(trap.max_velocity),
        unit_t<compound_unit<meters, inverse<squared<seconds>>>>(trap.max_acceleration));
    angle_pid = std::make_unique<frc::ProfiledPIDController<units::meters>>(
        pid.p, pid.i, pid.d,
        trap_const
    );
    angle_pid->SetTolerance(units::meter_t{1.0});
}

void MK4SwerveModule::setDrivePID(const PIDConfig& pid)
{
    drive_pid = std::make_unique<frc::PIDController>(
        pid.p, pid.i, pid.d
    );
}


void MK4SwerveModule::setAngleFeedForward(const FeedForwardConfig& ffc)
{
    angle_feedforward = std::make_unique<frc::SimpleMotorFeedforward<units::meters>>(
        units::volt_t{ffc.k_s},
        units::unit_t<frc::SimpleMotorFeedforward<units::meters>::kv_unit>(ffc.k_v),
        units::unit_t<frc::SimpleMotorFeedforward<units::meters>::ka_unit>(ffc.k_a)
    );
}

void MK4SwerveModule::setDriveFeedForward(const FeedForwardConfig& ffc)
{
    drive_feedforward = std::make_unique<frc::SimpleMotorFeedforward<units::meters>>(
            units::volt_t{ffc.k_s},
            units::unit_t<frc::SimpleMotorFeedforward<units::meters>::kv_unit>(ffc.k_v),
            units::unit_t<frc::SimpleMotorFeedforward<units::meters>::ka_unit>(ffc.k_a)
    );
}

void MK4SwerveModule::resetError()
{
    angle_pid->Reset(units::meter_t{0});
    drive_speed_max = 0.0;
}


double MK4SwerveModule::getFalconEncoderPosition()
{
    return drive_motor->GetPosition().GetValue().value();
}

//Assumes that the motor encoders are already returning distance in meters
//You can set this using the Phoenix X tool
frc::SwerveModulePosition MK4SwerveModule::getModulePosition()
{
    frc::SwerveModulePosition result;
    result.angle = units::degree_t{getAngle()};
    result.distance = units::meter_t{getFalconEncoderPosition()};

    return result;
}

void MK4SwerveModule::setMaxSpeed(double speed_in_meters_per_second)
{
    max_drive_speed = speed_in_meters_per_second;
}

void MK4SwerveModule::setToBrake()
{
    rotate_motor->SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
    drive_motor->SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
}

void MK4SwerveModule::setToCoast()
{
    rotate_motor->SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast);
    drive_motor->SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast);
}
