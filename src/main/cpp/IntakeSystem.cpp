#include "IntakeSystem.h"

IntakeSystem::IntakeSystem(std::shared_ptr<RobotContext> context):
    context(context)
{
    auto config = context->getConfig();
    intake_motor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(config.intake_config.intake_motor_id);
    intake_motor->SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast);

    linebreak = std::make_unique<frc::DigitalInput>(0);
    trap_servo = std::make_unique<frc::Servo>(config.intake_config.servo_pwm_id);

    frc::SmartDashboard::SetDefaultBoolean("Note secure?", false);
}

void IntakeSystem::Periodic()
{
    switch(mode)
    {
        case IntakeMode::STOP:
            doStop();
            break;
        case IntakeMode::FEED:
            doFeed();
            break;
        case IntakeMode::REVERSE:
            doReverse();
            break;
        case IntakeMode::INTAKE:
            doIntake();
            break;
    }

    frc::SmartDashboard::PutBoolean("Note secure?", !linebreak->Get());
    //intake_motor->SetVoltage(units::volt_t{-10.0});
    
    //std::cout << std::boolalpha << linebreak->Get() << std::endl;

}

void IntakeSystem::stop()
{
    mode = IntakeMode::STOP;
}

void IntakeSystem::doStop()
{
    intake_motor->StopMotor();
}

void IntakeSystem::feed()
{
    mode = IntakeMode::FEED;
}

void IntakeSystem::doFeed()
{
    intake_motor->SetVoltage(feed_voltage);
}

void IntakeSystem::reverse()
{
    mode = IntakeMode::REVERSE;
}

void IntakeSystem::doReverse()
{
    intake_motor->SetVoltage(reverse_voltage);
}

void IntakeSystem::intake()
{
    if(mode != IntakeMode::INTAKE || line_break_debounce.getValue())
    {
        mode = IntakeMode::INTAKE;
        line_break_debounce.forceValue(false);
    }
}

void IntakeSystem::doIntake()
{
    line_break_debounce.setValue(!linebreak->Get()); //linebreak true when see no object
    bool is_triggered = line_break_debounce.getValue();
    if(!linebreak->Get())
    {
        mode = IntakeMode::STOP;
        intake_motor->StopMotor();
        return;
    }

    intake_motor->SetVoltage(intake_voltage);
}

void IntakeSystem::setIntakeVoltage(units::volt_t voltage)
{
    intake_voltage = voltage;
}

void IntakeSystem::setIntakeVoltageLua(double voltage)
{
    setIntakeVoltage(units::volt_t{voltage});
}

bool IntakeSystem::isRingSecure()
{
    return !linebreak->Get();
}

void IntakeSystem::setTrapServoAngle(double angle)
{
    trap_servo->Set(angle / MAX_TRAP_SERVO_ANGLE);
}

void IntakeSystem::addToLuaState(Keeko::ThreadedScript& script)
{
    luabridge::getGlobalNamespace(script)
        .beginClass<IntakeSystem> ("IntakeSystem")
            .addFunction("stop", &IntakeSystem::stop)
            .addFunction("feed", &IntakeSystem::feed)
            .addFunction("reverse", &IntakeSystem::reverse)
            .addFunction("intake", &IntakeSystem::intake)
            .addFunction("hasRing", &IntakeSystem::isRingSecure)
            .addFunction("setVoltage", &IntakeSystem::setIntakeVoltageLua)
        .endClass();
    script.addInstance("intake_system", this);
}

