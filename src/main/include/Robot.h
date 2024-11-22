// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>
#include <map>
#include <thread>
#include <atomic>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/DriverStation.h>
#include <frc/Watchdog.h>

#include "RobotContext.h"
#include "DriveSystem.h"
#include "ShooterSystem.h"
#include "IntakeSystem.h"
#include "ClimbSystem.h"
#include "Paths.h"
#include "Lights.h"

#include <Keeko/LuaException.h>
#include <Keeko/Scriptable.h>
#include <Keeko/ThreadedScript.h>

//Third Party
#include "lua.h"
#include "lauxlib.h"
#include "lualib.h"
#include "LuaBridge.h"

class Robot : public frc::TimedRobot {
public:
    void RobotInit() override;
    void RobotPeriodic() override;
    void AutonomousInit() override;
    void AutonomousPeriodic() override;
    void TeleopInit() override;
    void TeleopPeriodic() override;
    void DisabledInit() override;
    void DisabledPeriodic() override;
    void TestInit() override;
    void TestPeriodic() override;
    void SimulationInit() override;
    void SimulationPeriodic() override;

    //Lua functions
    std::string getAllianceColor();
    double getTimeLeft();

    std::map<std::string, std::filesystem::path> auto_file_map
    {
        {"test", Paths::TEST_AUTON},
        {"3 ring left", Paths::THREE_RING_LEFT_AUTON},
        {"5 ring", Paths::FIVE_RING_AUTON},
        {"Center Clear", Paths::CENTER_CLEAR_LUA},
        {"four ring near 2", Paths::FOUR_RING_NEAR_AUTON_2},
        {"center field", Paths::CENTER_FIELD_AUTON}, 
        {"one shot", Paths::ONE_SHOT_AUTON}
    };

    enum class SysIdMode
    {
        DRIVE,
        ROTATE,
        ARM,
        SHOOTER
    };

    std::map<std::string, SysIdMode> SYSID_MODE_MAP
    {
        {"Drive motors", SysIdMode::DRIVE},
        {"Rotate motors", SysIdMode::ROTATE},
        {"Arm motors", SysIdMode::ARM},
        {"Shooter motors", SysIdMode::SHOOTER}
    };

    static constexpr double SLOW_SHOOTING_SPEED_ANGLE{-15.0};


private:
    frc::SendableChooser<std::string> m_chooser;
    frc::SendableChooser<std::string> sys_id_chooser;
    frc::SendableChooser<bool> sys_id_enabled_chooser;
    frc::SendableChooser<SysIdMode> sys_id_mode_chooser;
    frc::SendableChooser<bool> button_board_mode_chooser;
    
    const std::vector<std::string> sys_id_type
    {
        {
            "quasistatic forward",
            "quasistatic reverse",
            "dynamic forward",
            "dynamic reverse"
        }
    };
    const std::string kAutoNameDefault = "Default";
    const std::string kAutoNameCustom = "My Auto";
    std::string m_autoSelected;
    
    std::shared_ptr<RobotContext> robot_context;
    std::unique_ptr<DriveSystem> drive_system;
    std::unique_ptr<ShooterSystem> shooter_system;
    std::unique_ptr<IntakeSystem> intake_system;
    std::unique_ptr<ClimbSystem> climb_system;
    std::unique_ptr<std::thread> config_thread;
    std::unique_ptr<std::thread> lua_load_thread;
    Lights lights;
    std::atomic<bool> is_loading_script{false};

    void addToLuaState(Keeko::ThreadedScript& script);

    std::unique_ptr<Keeko::ThreadedScript> auton_script;
    std::optional<frc2::CommandPtr> sysid_command;
    bool last_shooter_input{false};
    bool last_driver_lob_shooter_input{false};
    bool lob_running{false};
    bool shooter_running{false};
};