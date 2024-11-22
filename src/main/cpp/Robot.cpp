// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <iostream>

#include "Robot.h"

#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <nlohmann/json.hpp>

void Robot::RobotInit()
{

    bool is_first_auto{true};
    for(const auto& auto_file_map_pair: auto_file_map)
    {
        auto auton_name = auto_file_map_pair.first;
        if(is_first_auto)
        {
            m_chooser.SetDefaultOption(auton_name, auton_name);
            is_first_auto = false;
        }
        m_chooser.AddOption(auton_name, auton_name);
    }


    sys_id_chooser.SetDefaultOption("idle", "idle");
    for(const auto& mode : sys_id_type)
    {
        sys_id_chooser.AddOption(mode, mode);
    }

    sys_id_mode_chooser.SetDefaultOption("Drive motors", SysIdMode::DRIVE);
    for(const auto& mode: SYSID_MODE_MAP)
    {
        sys_id_mode_chooser.AddOption(mode.first, mode.second);
    }
    

    sys_id_enabled_chooser.SetDefaultOption("disabled", false);
    sys_id_enabled_chooser.AddOption("disabled", false);
    sys_id_enabled_chooser.AddOption("enabled", true);

    button_board_mode_chooser.SetDefaultOption("xbox", true);
    button_board_mode_chooser.AddOption("xbox", true);
    button_board_mode_chooser.AddOption("switch", false);


    frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
    frc::SmartDashboard::PutData("Sys ID Type", &sys_id_chooser);
    frc::SmartDashboard::PutData("Sys ID Enabled", &sys_id_enabled_chooser);
    frc::SmartDashboard::PutData("Sys ID mechanism", &sys_id_mode_chooser);
    frc::SmartDashboard::PutData("Button Board Mode", &button_board_mode_chooser);
    //frc::SmartDashboard::PutData("Note Secure?", )


    
    robot_context = std::make_shared<RobotContext>();
    robot_context->loadConfig();
    // config_thread = std::make_unique<std::thread>([this]()
    // {
    //     try
    //     {
    //         robot_context->loadConfig();
    //     }
    //     catch(const std::exception& e)
    //     {
    //         std::cerr << e.what() << '\n';
    //     }
    // });
    
    
    drive_system = std::make_unique<DriveSystem>(robot_context);
    shooter_system = std::make_unique<ShooterSystem>(robot_context);
    intake_system = std::make_unique<IntakeSystem>(robot_context);
    climb_system = std::make_unique<ClimbSystem>(robot_context);
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic()
{

}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit()
{
    robot_context->loadConfig();
    drive_system->loadFromConfig();
    drive_system->reset();
    drive_system->setToBrake();
    shooter_system->setArmBreak(true);
    robot_context->startAngle = -robot_context->getPigeon().GetYaw().GetValue().value();//TODO: Remove this once we get auto working. Replace with start angle action from user

    //Check for sysid
    bool sys_enabled_selection = sys_id_enabled_chooser.GetSelected();

    if(sys_enabled_selection)
    {
        shooter_system->setSysIdEnabled(true);
        drive_system->setDriveMode(DriveSystem::DriveMode::SYS_ID);
        std::string sys_id_selection = sys_id_chooser.GetSelected();
        auto sys_id_mode_selection = sys_id_mode_chooser.GetSelected();
        switch(sys_id_mode_selection)
        {
            case SysIdMode::DRIVE:
            {
                if(sys_id_selection == "quasistatic forward")
                {
                    sysid_command = drive_system->getDriveSysIdCommand(DriveSystem::SysIdMode::QUASISTATIC_FORWARD); 
                }
                else if(sys_id_selection == "quasistatic reverse")
                {
                    sysid_command = drive_system->getDriveSysIdCommand(DriveSystem::SysIdMode::QUASISTATIC_REVERSE);                 
                }
                else if(sys_id_selection == "dynamic forward")
                {
                    sysid_command = drive_system->getDriveSysIdCommand(DriveSystem::SysIdMode::DYNAMIC_FORWARD);                    
                }
                else if(sys_id_selection == "dynamic reverse")
                {
                    sysid_command = drive_system->getDriveSysIdCommand(DriveSystem::SysIdMode::DYNAMIC_REVERSE);                    
                }
                if(sysid_command.has_value())
                {
                    sysid_command.value().Schedule();
                }
            }break;
            case SysIdMode::ROTATE:
            {
                if(sys_id_selection == "quasistatic forward")
                {
                    sysid_command = drive_system->getRotateSysIdCommand(DriveSystem::SysIdMode::QUASISTATIC_FORWARD); 
                }
                else if(sys_id_selection == "quasistatic reverse")
                {
                    sysid_command = drive_system->getRotateSysIdCommand(DriveSystem::SysIdMode::QUASISTATIC_REVERSE);                 
                }
                else if(sys_id_selection == "dynamic forward")
                {
                    sysid_command = drive_system->getRotateSysIdCommand(DriveSystem::SysIdMode::DYNAMIC_FORWARD);                    
                }
                else if(sys_id_selection == "dynamic reverse")
                {
                    sysid_command = drive_system->getRotateSysIdCommand(DriveSystem::SysIdMode::DYNAMIC_REVERSE);                    
                }
                if(sysid_command.has_value())
                {
                    sysid_command.value().Schedule();
                }
            }break;
            case SysIdMode::ARM:
            {
                if(sys_id_selection == "quasistatic forward")
                {
                    sysid_command = shooter_system->getArmSysIdCommand(ShooterSystem::SysIdMode::QUASISTATIC_FORWARD); 
                }
                else if(sys_id_selection == "quasistatic reverse")
                {
                    sysid_command = shooter_system->getArmSysIdCommand(ShooterSystem::SysIdMode::QUASISTATIC_REVERSE);                 
                }
                else if(sys_id_selection == "dynamic forward")
                {
                    sysid_command = shooter_system->getArmSysIdCommand(ShooterSystem::SysIdMode::DYNAMIC_FORWARD);                    
                }
                else if(sys_id_selection == "dynamic reverse")
                {
                    sysid_command = shooter_system->getArmSysIdCommand(ShooterSystem::SysIdMode::DYNAMIC_REVERSE);                    
                }
                if(sysid_command.has_value())
                {
                    sysid_command.value().Schedule();
                }
            }break;
            case SysIdMode::SHOOTER:
            {
                if(sys_id_selection == "quasistatic forward")
                {
                    sysid_command = shooter_system->getShooterSysIdCommand(ShooterSystem::SysIdMode::QUASISTATIC_FORWARD); 
                }
                else if(sys_id_selection == "quasistatic reverse")
                {
                    sysid_command = shooter_system->getShooterSysIdCommand(ShooterSystem::SysIdMode::QUASISTATIC_REVERSE);                 
                }
                else if(sys_id_selection == "dynamic forward")
                {
                    sysid_command = shooter_system->getShooterSysIdCommand(ShooterSystem::SysIdMode::DYNAMIC_FORWARD);                    
                }
                else if(sys_id_selection == "dynamic reverse")
                {
                    sysid_command = shooter_system->getShooterSysIdCommand(ShooterSystem::SysIdMode::DYNAMIC_REVERSE);                    
                }
                if(sysid_command.has_value())
                {
                    sysid_command.value().Schedule();
                }
            }break;
            default:
            {
                std::cout << "Invalid sysid mode selected!" << std::endl;
            }
        }
    }
    else if(auton_script)
    {
        shooter_system->setSysIdEnabled(false);
        auton_script->runFile(auto_file_map[m_autoSelected].generic_string());
    }

    drive_system->setSpeakerPose(getAllianceColor());
    robot_context->getVision().setToAutoMode();
    robot_context->getVision().LEDOn();
}

void Robot::AutonomousPeriodic() {
    if(m_autoSelected == kAutoNameCustom)
    {
        // Custom Auto goes here
    }
    else 
    {
        // Default Auto goes here
    }
    frc2::CommandScheduler::GetInstance().Run();
    lights.updateLights();
}

void Robot::TeleopInit()
{
    //TODO: Load config on request and not in teleop init
    robot_context->loadConfig();
    drive_system->loadFromConfig();
    drive_system->reset();
    drive_system->setToBrake();
    drive_system->setDriveMode(DriveSystem::DriveMode::MANUAL);
    drive_system->setMaxDriveSpeed(4.0);//Move to config
    intake_system->stop();
    shooter_system->setSysIdEnabled(false);
    shooter_system->setArmBreak(true);
    frc2::CommandScheduler::GetInstance().SetPeriod(units::time::second_t{0.02});
    //Watchdog stuff
    robot_context->startAngle = -robot_context->getPigeon().GetYaw().GetValue().value();//TODO: Remove this once we get auto working. Replace with start angle action from user
    auto& button_board = robot_context->getButtonBoard();
    bool xbox_mode = button_board_mode_chooser.GetSelected();
    button_board.setIsXbox(xbox_mode);

    drive_system->setSpeakerPose(getAllianceColor());
    robot_context->getVision().setToDriverMode();
    robot_context->getVision().LEDOff();
    intake_system->setIntakeVoltage(units::volt_t{-7.0});
    shooter_system->setTargetArmAngle(-10.0);
}

void Robot::TeleopPeriodic()
{
    auto& pilot = robot_context->getPilotController();
    auto& button_board = robot_context->getButtonBoard();
    auto& copilot_controller = robot_context->getCopilotController();
    
    //drive 
    if(pilot.GetR2Button())
    {
        robot_context->getVision().LEDOn();
        drive_system->autoAim(pilot.GetLeftX(), pilot.GetLeftY());
    }
    else if(pilot.GetR1Button())
    {
        robot_context->getVision().LEDOff();
        drive_system->ringLock(pilot.GetLeftY());
    }
    else
    {
        robot_context->getVision().LEDOff();
        drive_system->setFieldOriented(true);
        drive_system->setDriveMode(DriveSystem::DriveMode::MANUAL);
        double translate_x = pilot.GetLeftX();
        double translate_y = pilot.GetLeftY();
        double turn = pilot.GetRightX();
        drive_system->setDrive(translate_x, translate_y, turn);
    }
    if(pilot.GetR3Button())
    {
        robot_context->startAngle = robot_context->getPigeon().GetYaw().GetValue().value();
        drive_system->reset();
    }

    //intake
    if(button_board.getLB() || copilot_controller.GetSquareButton())//Intake
    {
        shooter_system->setArmVoltage(units::volt_t(0.0));
        intake_system->intake();
    }
    else if((button_board.getY() || copilot_controller.GetCrossButton()) && shooter_system->isShooterRunning())//Feed
    {
        intake_system->feed();
    }
    else if(button_board.getLT() || copilot_controller.GetTriangleButton())//Reverse
    {
        intake_system->reverse();
    }
    else
    {
        intake_system->stop();
    }
    if(pilot.GetTriangleButton())
    {
        robot_context->getVision().takeSnapshot();
    }
    //shooter
    bool current_shooter_input = button_board.getX() || copilot_controller.GetCircleButton();
    bool driver_shooter_lob_input = pilot.GetL2Button();
    if(last_shooter_input == false && current_shooter_input)//On rising edge
    {
        shooter_running = !shooter_running; //Invert the value
    }
    if(last_driver_lob_shooter_input == false && driver_shooter_lob_input)//On rising edge
    {
        //lob_running = !lob_running;
    }
    if(shooter_running)
    {
        if(lob_running)
        {
            shooter_system->setTargetSpeed(ShooterSystem::LOB_SPEED);
        }
        else
        {
            if(shooter_system->getArmAngle() < SLOW_SHOOTING_SPEED_ANGLE)
            {
                shooter_system->setTargetSpeed(ShooterSystem::SLOW_SHOOTING_SPEED);
            }
            else
            {
                shooter_system->setTargetSpeed(ShooterSystem::SHOOTING_SPEED);
            }
        }
    }
    else
    {
        shooter_system->setTargetSpeed(0.0);
    }
    last_shooter_input = current_shooter_input;
    last_driver_lob_shooter_input = driver_shooter_lob_input;

    //climb
    if(pilot.GetL1Button())
    {
        climb_system->setClimbVoltage(ClimbSystem::UP_VOLTAGE);
    }
    else if(pilot.GetL2Button())
    {
        climb_system->setClimbVoltage(ClimbSystem::DOWN_VOLTAGE);
    }
    else
    {
        climb_system->setClimbVoltage(units::volt_t{0.0});
    }
    
    
    //arm
    auto arm_copilot_axis = Utility::deadZone(copilot_controller.GetLeftY(), 0.08);
    if(button_board.getRB() || copilot_controller.GetPOV() == 180)
    {
        shooter_system->setTargetArmAngle(ShooterSystem::ARM_FLOOR_ANGLE);
    }
    else if(button_board.getA() || copilot_controller.GetPOV() == 270)
    {
        shooter_system->setTargetArmAngle(ShooterSystem::ARM_AMP_ANGLE);
    }
    else if(button_board.getRT() || copilot_controller.GetPOV() == 0)
    {
        shooter_system->setTargetArmAngle(ShooterSystem::ARM_TOP_ANGLE);
    }
    else if(button_board.getB() || copilot_controller.GetPOV() == 90)
    {
        shooter_system->setTargetArmAngle(ShooterSystem::ARM_POST_ANGLE);
    }
    else if(std::abs(arm_copilot_axis) > 0.5)//Copilot axis control on gamepad
    {
        shooter_system->setArmVoltage(ShooterSystem::ARM_MANUAL_UP_VOLTAGE*arm_copilot_axis);
    }
    else if(button_board.getPOV() == 0)
    {
        shooter_system->setArmVoltage(ShooterSystem::ARM_MANUAL_UP_VOLTAGE);
    }
    else if(button_board.getPOV() == 180)
    {
        shooter_system->setArmVoltage(ShooterSystem::ARM_MANUAL_DOWN_VOLTAGE);
    }
    else if(button_board.getPOV() == -1 && !shooter_system->isArmPositionMode())
    {
        shooter_system->setArmVoltage(units::volt_t{0.0});
    }
    drive_system->updateAnglesSMD();

    //lights
    // if(drive_system->speakerLockedOn())
    // {
    //     lights.jitter(
    //         Lights::GREEN,
    //         Lights::ORANGE,
    //         3,
    //         8);
    // }
    if(intake_system->isRingSecure())
    {
        if(shooter_system->isShooterRunning())
        {
            if(lob_running)
            {
                lights.sweep(
                    Lights::GREEN,
                    Lights::ORANGE,
                    6,
                    -8
                );
            }
            else
            {
                lights.sweep(
                    Lights::GREEN,
                    Lights::ORANGE,
                    6,
                    16
                );
            }
        }
        else
        {
            lights.solid(Lights::GREEN);
        }
    }
    else if(shooter_system->isShooterRunning())
    {
        if(lob_running)
        {
            lights.sweep(
                Lights::ORANGE,
                Lights::BLACK,
                6,
                -8
            );
        }
        else
        {
            lights.sweep(
                Lights::ORANGE,
                Lights::BLACK,
                6,
                16
            );
        }
    }
    else
    {
        lights.solid(Lights::BLACK);
    }


    frc2::CommandScheduler::GetInstance().Run();
    shooter_system->updateSMD();

    lights.updateLights();
}

void Robot::DisabledInit() 
{
    drive_system->setToCoast();
    if(sysid_command.has_value())
    {
        sysid_command.value().Cancel();
        sysid_command.reset();
    }
    shooter_system->setSysIdEnabled(true);
    shooter_system->setArmBreak(false);
    robot_context->getVision().LEDOff();
}

void Robot::DisabledPeriodic()
{
    drive_system->updateAnglesSMD();
    shooter_system->updateSMD();   
    if(auton_script)
    {
        auton_script->stop();
    }
    if(m_autoSelected != m_chooser.GetSelected() && !is_loading_script)
    {
        if(lua_load_thread && lua_load_thread->joinable())
        {
            lua_load_thread->join();
        }
        
        lua_load_thread = std::make_unique<std::thread>([this]()
        {
            is_loading_script = true;
            m_autoSelected = m_chooser.GetSelected();
            // m_autoSelected = SmartDashboard::GetString("Auto Selector",
            //     kAutoNameDefault);
            fmt::print("Auto selected: {}\n", m_autoSelected);
            if(auto_file_map.find(m_autoSelected) == auto_file_map.end())
            {
                std::cout << "auton not found in map";
                is_loading_script = false;
                return;
            }
            auton_script = std::make_unique<Keeko::ThreadedScript>();
            drive_system->addToLuaState(*auton_script);
            intake_system->addToLuaState(*auton_script);
            shooter_system->addToLuaState(*auton_script);
            robot_context->getVision().addToLuaState(*auton_script);
            addToLuaState(*auton_script);
            auton_script->setOnFailCallback([this]()
            {
                std::cout << "Script failed! Stopping subsytems." << std::endl;
                drive_system->stop();
                shooter_system->setArmVoltage(units::volt_t{0});
                shooter_system->setTargetSpeed(0);
                intake_system->stop();
                climb_system->setClimbVoltage(units::volt_t{0});
            });
            auton_script->loadFile(Paths::SHARED_LUA.generic_string());
            std::cout << "Script is ready to run! We have selected " << m_autoSelected << std::endl;
            is_loading_script = false;
        });
    }
    auto alliance = frc::DriverStation::GetAlliance();
    if(alliance.has_value())
    {
        if(alliance.value() == frc::DriverStation::Alliance::kBlue)
        {
            lights.sweep(
                Lights::BLUE,
                Lights::BLACK,
                Lights::NUM_LIGHTS/2,
                10
            );
        }
        else
        {
            lights.sweep(
                Lights::RED,
                Lights::BLACK,
                Lights::NUM_LIGHTS/2,
                10
            );
        }
    }
    else //Robot not connected to a driver station, this is good for debugging field issues while in the stands
    {
        lights.sweep(
            Lights::RED,
            Lights::BLUE,
            Lights::NUM_LIGHTS/2,
            10
        );
    }

    lights.updateLights();
    robot_context->getVision().LEDOff();
    robot_context->getVision().setToAutoMode();
}

void Robot::TestInit()
{
}

void Robot::TestPeriodic() {}

void Robot::SimulationInit()
{
    using namespace units;
    std::cout << "----==== Simulation init ====----" << std::endl;
}

void Robot::SimulationPeriodic() {}

std::string Robot::getAllianceColor()
{
    auto alliance = frc::DriverStation::GetAlliance();
    if(alliance.has_value())
    {
        switch(alliance.value())
        {
            case frc::DriverStation::Alliance::kBlue: return "blue";
            case frc::DriverStation::Alliance::kRed: return "red";
        }
    }
    return "invalid";
}

double Robot::getTimeLeft()
{
    return frc::DriverStation::GetMatchTime().value();
}

void Robot::addToLuaState(Keeko::ThreadedScript& script)
{
    luabridge::getGlobalNamespace(script)
    .beginClass<Robot> ("RobotClass")
        .addFunction("getAllianceColor", &Robot::getAllianceColor)
        .addFunction("getTimeLeft", &Robot::getTimeLeft)
    .endClass();
    script.addInstance("robot", this);
}


#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif
