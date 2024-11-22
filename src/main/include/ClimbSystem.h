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

//Our Lib
#include "RobotContext.h"
#include <Keeko/LuaException.h>
#include <Keeko/Scriptable.h>
#include <Keeko/ThreadedScript.h>

//Third Party
#include "lua.h"
#include "lauxlib.h"
#include "lualib.h"
#include "LuaBridge.h"

class ClimbSystem: public frc2::SubsystemBase
{
public:
    ClimbSystem() = delete;
    ClimbSystem(const ClimbSystem&) = delete;
    ClimbSystem(std::shared_ptr<RobotContext> context);

    void setClimbVoltage(units::volt_t volts);

    static constexpr units::volt_t UP_VOLTAGE{12.0};
    static constexpr units::volt_t DOWN_VOLTAGE{-8.0};

private:
    void Periodic() override;

    std::shared_ptr<RobotContext> context;
    std::unique_ptr<ctre::phoenix6::hardware::TalonFX> left_elevator_motor;
    std::unique_ptr<ctre::phoenix6::hardware::TalonFX> right_elevator_motor;
    units::volt_t climb_voltage{0.0};
};