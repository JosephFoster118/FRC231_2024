#include "ClimbSystem.h"

ClimbSystem::ClimbSystem(std::shared_ptr<RobotContext> context):
    context(context)
{
    auto config = context->getConfig();
    left_elevator_motor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(config.climb_config.elevator_left_id);
    right_elevator_motor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(config.climb_config.elevator_right_id);

    left_elevator_motor->SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
    right_elevator_motor->SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);

}

void ClimbSystem::Periodic()
{
    left_elevator_motor->SetVoltage(-climb_voltage);
    right_elevator_motor->SetVoltage(climb_voltage);
}

void ClimbSystem::setClimbVoltage(units::volt_t volts)
{
    climb_voltage = volts;
}
