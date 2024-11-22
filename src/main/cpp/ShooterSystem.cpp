#include "ShooterSystem.h"

ShooterSystem::ShooterSystem(std::shared_ptr<RobotContext> context):
    context(context)
{
    auto config = context->getConfig();

    top_shooter_motor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(config.shooter_config.shooter_top_id);
    bottom_shooter_motor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(config.shooter_config.shooter_bottom_id);
    left_arm_motor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(config.climb_config.arm_left_id);
    right_arm_motor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(config.climb_config.arm_right_id);
    // left_arm_motor->SetInverted(true);
    // right_arm_motor->SetInverted(true);
    left_arm_motor->SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
    right_arm_motor->SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
    top_shooter_motor->SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast);
    bottom_shooter_motor->SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast);
    shooter_pid = std::make_unique<frc::PIDController>(
        config.shooter_config.shooter_pid.p,
        config.shooter_config.shooter_pid.i,
        config.shooter_config.shooter_pid.d
    );
    shooter_feedforward = std::make_unique<frc::SimpleMotorFeedforward<units::meters>>(
        units::volt_t{config.shooter_config.shooter_feedforward.k_s},
        units::unit_t<frc::SimpleMotorFeedforward<units::meters>::kv_unit>(config.shooter_config.shooter_feedforward.k_v),
        units::unit_t<frc::SimpleMotorFeedforward<units::meters>::ka_unit>(config.shooter_config.shooter_feedforward.k_a)
    );

    arm_pid = std::make_unique<frc::PIDController>(
        config.shooter_config.arm_pid.p,
        config.shooter_config.arm_pid.i,
        config.shooter_config.arm_pid.d
    );

    arm_feedforward = std::make_unique<frc::ArmFeedforward>(
        units::volt_t{config.shooter_config.arm_feedforward.k_s},
        units::volt_t{config.shooter_config.arm_feedforward.k_g},
        units::unit_t<units::compound_unit<units::volts, units::inverse<units::degrees_per_second>>>{config.shooter_config.arm_feedforward.k_v},
        units::unit_t<units::compound_unit<units::volts, units::inverse<units::degrees_per_second_squared>>>{config.shooter_config.arm_feedforward.k_a}
    );


    arm_encoder = std::make_unique<ctre::phoenix6::hardware::CANcoder>(config.climb_config.arm_encoder_id);
    arm_encoder_offset = config.climb_config.arm_encoder_offset;

    sysid_arm_routine = std::make_unique<frc2::sysid::SysIdRoutine>(
        frc2::sysid::Config{frc2::sysid::ramp_rate_t{0.4}, units::volt_t{3.5}, units::second_t{10},
                          std::nullopt},
        frc2::sysid::Mechanism
        {
            [this](units::volt_t drive_voltage)
            {
                left_arm_motor->SetVoltage(-drive_voltage);
                right_arm_motor->SetVoltage(drive_voltage);
            },
            [this](frc::sysid::SysIdRoutineLog* log)
            {
                log->Motor("arm")
                    .voltage(right_arm_motor->GetMotorVoltage().GetValue())
                    .position(units::meter_t{getArmAngle()})
                    .velocity(units::meters_per_second_t{arm_encoder->GetVelocity().GetValueAsDouble()*360.0});//TODO: May need to invert
            },
            this
        }
    );

    sysid_shooter_routine = std::make_unique<frc2::sysid::SysIdRoutine>(
        frc2::sysid::Config{std::nullopt, std::nullopt, std::nullopt,
                          std::nullopt},
        frc2::sysid::Mechanism
        {
            [this](units::volt_t drive_voltage)
            {
                top_shooter_motor->SetVoltage(drive_voltage);
                bottom_shooter_motor->SetVoltage(drive_voltage);
            },
            [this](frc::sysid::SysIdRoutineLog* log)
            {
                log->Motor("arm")
                    .voltage(top_shooter_motor->GetMotorVoltage().GetValue())
                    .position(units::meter_t{top_shooter_motor->GetPosition().GetValueAsDouble()})
                    .velocity(units::meters_per_second_t{top_shooter_motor->GetVelocity().GetValueAsDouble()*360.0});//TODO: May need to invert
            },
            this
        }
    );

    frc::SmartDashboard::SetDefaultNumber(SHOOTER_SPEED_KEY, 0.0);
    frc::SmartDashboard::SetDefaultNumber("Arm Raw Angle", 0.0);
    frc::SmartDashboard::SetDefaultNumber("Arm Angle", 0.0);
    frc::SmartDashboard::SetDefaultBoolean("Shooter running", false);
    frc::SmartDashboard::SetDefaultNumber("Shooter voltage", 0.0);
}   

void ShooterSystem::Periodic()
{
    if(!sysid_enabled)
    {
        auto current_speed = getCurrentSpeed();
        auto shooter_pid_value = units::volt_t{shooter_pid->Calculate(current_speed, target_speed)};
        auto shooter_feedforward_value = shooter_feedforward->Calculate(
            units::unit_t<units::compound_unit<units::meters, units::inverse<units::minutes>>>{target_speed}
        );
        // top_shooter_motor->SetVoltage(shooter_pid_value + shooter_feedforward_value);
        // bottom_shooter_motor->SetVoltage(shooter_pid_value + shooter_feedforward_value);
        top_shooter_motor->SetVoltage(-units::volt_t{target_speed});
        bottom_shooter_motor->SetVoltage(-units::volt_t{target_speed});

        //arm
        if(arm_position_mode)
        {
            auto current_angle = getArmAngle();
            auto arm_pid_value = arm_pid->Calculate(Utility::closestAngle(target_arm_angle, current_angle), 0.0);
            frc::SmartDashboard::PutNumber(SHOOTER_SPEED_KEY, arm_pid_value);
            arm_pid_value = Utility::minMax(-12.0, 12.0, arm_pid_value);\
            if(arm_pid_value < 0.0)
            {
                arm_pid_value *= 0.8;
            }
            // auto arm_ff = arm_feedforward->Calculate(
            //     units::degree_t{target_arm_angle},
            //     units::angular_velocity::degrees_per_second_t{arm_pid_value}
            // );
            left_arm_motor->SetVoltage(-units::volt_t{arm_pid_value});
            right_arm_motor->SetVoltage(units::volt_t{arm_pid_value});
        }
        else
        {
            frc::SmartDashboard::PutNumber(SHOOTER_SPEED_KEY, -231);
            left_arm_motor->SetVoltage(-arm_voltage);
            right_arm_motor->SetVoltage(arm_voltage);
        }
    }
    frc::SmartDashboard::PutBoolean("Shooter running", isShooterRunning());
    frc::SmartDashboard::PutNumber("Shooter voltage", target_speed);
}

void ShooterSystem::setTargetSpeed(double speed)
{
    target_speed = speed;
}
 
double ShooterSystem::getTargetSpeed()
{
    return target_speed;    
}

double ShooterSystem::getCurrentSpeed()
{
    return top_shooter_motor->GetVelocity().GetValueAsDouble();
}

void ShooterSystem::addToLuaState(Keeko::ThreadedScript& script)
{
    luabridge::getGlobalNamespace(script)
        .beginClass<ShooterSystem> ("ShooterSystem")
            .addFunction("setTargetSpeed", &ShooterSystem::setTargetSpeed)
            .addFunction("getTargetSpeed", &ShooterSystem::getTargetSpeed)
            .addFunction("getCurrentSpeed", &ShooterSystem::getCurrentSpeed)
            .addFunction("setTargetArmAngle", &ShooterSystem::setTargetArmAngle)
            .addFunction("setTargetSpeed", &ShooterSystem::setTargetSpeed)
            .addFunction("getArmAngle", &ShooterSystem::getArmAngle)
        .endClass();
    script.addInstance("shooter_system", this);
}

double ShooterSystem::getRawArmAngle()
{
    return Utility::get180Range(arm_encoder->GetPosition().GetValueAsDouble()*360.0);
}

double ShooterSystem::getArmAngle()
{
    return Utility::get180Range(getRawArmAngle() - arm_encoder_offset);
}

double ShooterSystem::getArm360Range()//To make sysid better
{
    auto angle = getArmAngle();
    if(angle < 0.0)
    {
        return angle + 360;
    }
    return angle;
}

void ShooterSystem::updateSMD()
{
    frc::SmartDashboard::PutNumber("Arm Angle", getArmAngle());
    frc::SmartDashboard::PutNumber("Arm Raw Angle", getRawArmAngle());
}

frc2::CommandPtr ShooterSystem::getArmSysIdCommand(SysIdMode mode)
{
    switch(mode)
    {
        case SysIdMode::QUASISTATIC_FORWARD: return sysid_arm_routine->Quasistatic(frc2::sysid::Direction::kForward);
        case SysIdMode::QUASISTATIC_REVERSE: return sysid_arm_routine->Quasistatic(frc2::sysid::Direction::kReverse);
        case SysIdMode::DYNAMIC_FORWARD: return sysid_arm_routine->Dynamic(frc2::sysid::Direction::kForward);
        case SysIdMode::DYNAMIC_REVERSE: return sysid_arm_routine->Dynamic(frc2::sysid::Direction::kReverse);
        default: return sysid_arm_routine->Dynamic(frc2::sysid::Direction::kReverse);
    }
}

frc2::CommandPtr ShooterSystem::getShooterSysIdCommand(SysIdMode mode)
{
    switch(mode)
    {
        case SysIdMode::QUASISTATIC_FORWARD: return sysid_shooter_routine->Quasistatic(frc2::sysid::Direction::kForward);
        case SysIdMode::QUASISTATIC_REVERSE: return sysid_shooter_routine->Quasistatic(frc2::sysid::Direction::kReverse);
        case SysIdMode::DYNAMIC_FORWARD: return sysid_shooter_routine->Dynamic(frc2::sysid::Direction::kForward);
        case SysIdMode::DYNAMIC_REVERSE: return sysid_shooter_routine->Dynamic(frc2::sysid::Direction::kReverse);
        default: return sysid_shooter_routine->Dynamic(frc2::sysid::Direction::kReverse);
    }
}

void ShooterSystem::setSysIdEnabled(bool enabled)
{
    sysid_enabled = enabled;
}

void ShooterSystem::setArmBreak(bool on)
{
    if(on != is_arm_braked)
    {
        if(on)
        {
            left_arm_motor->SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
            right_arm_motor->SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
        }
        else
        {
            left_arm_motor->SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast);
            right_arm_motor->SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast);
        }
    }
    is_arm_braked = on;
}
    
void ShooterSystem::setArmVoltage(units::volt_t volt)
{
    arm_position_mode = false;
    arm_voltage = volt;
}

void ShooterSystem::setTargetArmAngle(double angle)
{
    arm_position_mode = true;
    target_arm_angle = angle;
}

bool ShooterSystem::isArmPositionMode()
{
    return arm_position_mode;
}

bool ShooterSystem::isShooterRunning()
{
    return (target_speed > 2.0) || (target_speed < -2.0);
}

double ShooterSystem::autoAimArmAngle(std::optional<double> distance)
{
    if(!distance.has_value()) //if target not in view, put arm to the floor (for auto aiming while lobbing)
    {
        return ARM_FLOOR_ANGLE;
    }
    //return (get arm angle from fit line)
    return 0.0;
}
