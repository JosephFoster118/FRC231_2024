#pragma once

#include "RobotConfig.h"
#include "Paths.h"
#include <frc/XboxController.h>
#include <frc/PS4Controller.h>
#include <AHRS.h>
#include "PXN0082.h"
#include "Vision.h"
#include <memory>
#include <ctre/phoenix6/Pigeon2.hpp>

class RobotContext
{
public:
    void loadConfig();
    const RobotConfig& getConfig();

    const frc::PS4Controller& getPilotController();
    frc::PS4Controller& getCopilotController();
    PXN0082& getButtonBoard();
    //AHRS& getNavx();
    double startAngle{0.0};
    Vision& getVision();
    ctre::phoenix6::hardware::Pigeon2& getPigeon();

private:
    RobotConfig config;
    frc::PS4Controller pilot_controller{0};
    PXN0082 button_board{1};
    frc::PS4Controller copilot_controller{2};
    //AHRS navx{frc::SPI::Port::kMXP};
    Vision vision;
    ctre::phoenix6::hardware::Pigeon2 pigeon{0, "rio"};

};

