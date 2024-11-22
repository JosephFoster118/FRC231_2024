#pragma once
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp> 
#include <ctre/phoenix6/signals/SpnEnums.hpp>
#include <memory>

#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/PIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/angle.h>



#include "SwerveModule.h"
#include "PID.h"
#include "RobotConfig.h"

class MK4SwerveModule: public SwerveModule
{
public:
    MK4SwerveModule() = delete;
    MK4SwerveModule(MK4SwerveModule&) = delete;
    MK4SwerveModule(uint8_t drive_id, uint8_t rotate_id, uint8_t encoder_id, double encoder_offset, std::string module_name = "NA");

    double getAngle();
    double getRawAngle();
    double getEncoderDistance();
    void setMotors();
    void update(std::chrono::duration<double> seconds_passed);
    frc::SwerveModulePosition getModulePosition();
    void resetError();
    double getFalconEncoderPosition();
    void setToBrake();
    void setToCoast();

    void setAnglePID(const PIDConfig& pid, const TrapazoidalProfileConfig& trap);
    void setDrivePID(const PIDConfig& pid);

    void setAngleFeedForward(const FeedForwardConfig& ffc);
    void setDriveFeedForward(const FeedForwardConfig& ffc);

    void setMaxSpeed(double speed_in_meters_per_second);
    void setDriveVoltage(units::volt_t volts);
    void setRotateVoltage(units::volt_t volts);
    units::volt_t getDriveVoltage();
    units::meter_t getDrivePosition();
    units::meters_per_second_t getDriveVelocity();

    units::volt_t getRotateVoltage();
    units::meter_t getRotatePosition();
    units::meters_per_second_t getRotateVelocity();

    inline static constexpr double ALL_ENCODER_OFFSET{180.0}; //CHANGE TO 0 FOR COMP BOT

private:
    std::unique_ptr<ctre::phoenix6::hardware::TalonFX> drive_motor;
    std::unique_ptr<ctre::phoenix6::hardware::TalonFX> rotate_motor;
    std::unique_ptr<ctre::phoenix6::hardware::CANcoder> encoder;
    std::unique_ptr<frc::ProfiledPIDController<units::meters>> angle_pid;//TODO: Remove
    std::unique_ptr<frc::PIDController> drive_pid;
    std::unique_ptr<frc::SimpleMotorFeedforward<units::meters>> angle_feedforward;
    std::unique_ptr<frc::SimpleMotorFeedforward<units::meters>> drive_feedforward;
    double offset;
    double rotate_encoder_start;
    double can_encoder_start;
    double ticks_per_degree = 72.81778;
    double max_drive_speed = 4.0;
    double drive_speed_max = 0;
    std::string name;
};
