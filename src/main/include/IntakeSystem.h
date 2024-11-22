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
#include <frc/DigitalInput.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Servo.h>

//Our Lib
#include "RobotContext.h"
#include <Keeko/LuaException.h>
#include <Keeko/Scriptable.h>
#include <Keeko/ThreadedScript.h>
#include <Debounce.h>

//Third Party
#include "lua.h"
#include "lauxlib.h"
#include "lualib.h"
#include "LuaBridge.h"

class IntakeSystem: public frc2::SubsystemBase
{
public:
    IntakeSystem() = delete;
    IntakeSystem(const IntakeSystem&) = delete;
    IntakeSystem(std::shared_ptr<RobotContext> context);

    enum class IntakeMode
    {
        STOP,
        INTAKE,
        FEED,
        REVERSE
    };

    inline static std::string driveModeToString(IntakeMode mode)
    {
        switch(mode)
        {
            case IntakeMode::STOP: return "stop";
            case IntakeMode::INTAKE: return "intake";
            case IntakeMode::FEED: return "feed";
            case IntakeMode::REVERSE: return "reverse";
        }
    }

    inline static constexpr double MAX_TRAP_SERVO_ANGLE{270.0};

    void addToLuaState(Keeko::ThreadedScript& script);
    void stop();
    void doStop();
    void feed();
    void doFeed();
    void reverse();
    void doReverse();
    void intake();
    void doIntake();
    void setIntakeVoltage(units::volt_t voltage);
    void setIntakeVoltageLua(double voltage);
    bool isRingSecure();
    void setTrapServoAngle(double angle);

private:
    void Periodic() override;

    IntakeMode mode{IntakeMode::STOP};

    std::shared_ptr<RobotContext> context;
    std::unique_ptr<ctre::phoenix6::hardware::TalonFX> intake_motor;
    std::unique_ptr<frc::DigitalInput> linebreak;
    units::volt_t intake_voltage{-7.0};
    units::volt_t feed_voltage{-12.0};
    units::volt_t reverse_voltage{3.0};
    Debounce line_break_debounce{std::chrono::milliseconds(100)};
    std::unique_ptr<frc::Servo> trap_servo;
};
