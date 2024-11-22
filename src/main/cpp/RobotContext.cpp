#include "RobotContext.h"
#include <fstream>

void RobotContext::loadConfig()
{
    std::ifstream json_file(Paths::ROBOT_CONFIG);
    nlohmann::json j;
    json_file >> j;
    config = j;
}

const RobotConfig& RobotContext::getConfig()
{
    return config;
}

const frc::PS4Controller& RobotContext::getPilotController()
{
    return pilot_controller;
}

frc::PS4Controller& RobotContext::getCopilotController()
{
    return copilot_controller;
}

// AHRS& RobotContext::getNavx()
// {
//     return navx;
// }

PXN0082& RobotContext::getButtonBoard()
{
    return button_board;
}

Vision& RobotContext::getVision()
{
    return vision;
}

ctre::phoenix6::hardware::Pigeon2& RobotContext::getPigeon()
{
    return pigeon;
}
