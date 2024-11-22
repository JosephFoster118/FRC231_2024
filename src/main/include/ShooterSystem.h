#pragma once

//Standard Lib
#include <memory>
#include <chrono>
#include <mutex>

//Frc Lib
#include <frc2/command/Subsystem.h>
#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp> 
#include <ctre/phoenix6/signals/SpnEnums.hpp>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/PIDController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/sysid/SysIdRoutine.h>

//Our Lib
#include "RobotContext.h"
#include <Keeko/LuaException.h>
#include <Keeko/Scriptable.h>
#include <Keeko/ThreadedScript.h>
#include "Utility.h"

//Third Party
#include "lua.h"
#include "lauxlib.h"
#include "lualib.h"
#include "LuaBridge.h"

class ShooterSystem: public frc2::SubsystemBase
{
public:
    ShooterSystem() = delete;
    ShooterSystem(const ShooterSystem&) = delete;
    ShooterSystem(std::shared_ptr<RobotContext> context);

    enum class SysIdMode
    {
        QUASISTATIC_FORWARD,
        QUASISTATIC_REVERSE,
        DYNAMIC_FORWARD,
        DYNAMIC_REVERSE
    };

    void setTargetSpeed(double speed);
    double getTargetSpeed();
    void addToLuaState(Keeko::ThreadedScript& script);
    double getCurrentSpeed();
    double getTargetArmAngle();
    void setTargetArmAngle(double angle);
    void setSysIdEnabled(bool enabled);
    void setArmBreak(bool on);

    double getRawArmAngle();
    double getArmAngle();
    double autoAimArmAngle(std::optional<double> distance);
    double getArm360Range();
    bool isArmPositionMode();
    bool isShooterRunning();

    frc2::CommandPtr getArmSysIdCommand(SysIdMode mode);
    frc2::CommandPtr getShooterSysIdCommand(SysIdMode mode);
    void setArmVoltage(units::volt_t volt);
    void updateSMD();

    inline static const std::string SHOOTER_SPEED_KEY{"shooter speed"};
    inline static constexpr double ARM_FLOOR_ANGLE{-21.5};
    inline static constexpr double ARM_AMP_ANGLE{30.76};
    inline static constexpr double ARM_FARAWAY_ANGLE{-3.25};
    inline static constexpr double ARM_POST_ANGLE{-3.25};
    inline static constexpr double ARM_TOP_ANGLE{75};
    inline static constexpr double SHOOTING_SPEED{12.0};
    inline static constexpr double SLOW_SHOOTING_SPEED{10.0};
    inline static constexpr double LOB_SPEED{9.5};
    inline static constexpr units::volt_t ARM_MANUAL_UP_VOLTAGE{5.0};
    inline static constexpr units::volt_t ARM_MANUAL_DOWN_VOLTAGE{-5.0};


private:
    void Periodic() override;

    std::shared_ptr<RobotContext> context;
    std::unique_ptr<ctre::phoenix6::hardware::TalonFX> top_shooter_motor;
    std::unique_ptr<ctre::phoenix6::hardware::TalonFX> bottom_shooter_motor;
    std::unique_ptr<ctre::phoenix6::hardware::TalonFX> left_arm_motor;
    std::unique_ptr<ctre::phoenix6::hardware::TalonFX> right_arm_motor;
    std::unique_ptr<ctre::phoenix6::hardware::CANcoder> arm_encoder;
    std::unique_ptr<frc::PIDController> shooter_pid;
    std::unique_ptr<frc::SimpleMotorFeedforward<units::meters>> shooter_feedforward;
    std::unique_ptr<frc::PIDController> arm_pid;
    std::unique_ptr<frc::ArmFeedforward> arm_feedforward;
    std::unique_ptr<frc2::sysid::SysIdRoutine> sysid_arm_routine;
    std::unique_ptr<frc2::sysid::SysIdRoutine> sysid_shooter_routine;
    bool sysid_enabled{false};
    double target_speed;
    double target_arm_angle{90.0};
    units::volt_t arm_voltage{0.0};
    double arm_encoder_offset{0.0};
    bool arm_position_mode{false};
    bool is_arm_braked{false};
};