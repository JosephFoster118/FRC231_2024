#include "DriveSystem.h"

#include <iostream> //For debugging
#include <cmath>
#include <frc/Timer.h>

#include <frc/smartdashboard/SmartDashboard.h>

DriveSystem::DriveSystem(std::shared_ptr<RobotContext> context):
    context(context),
    frc2::SubsystemBase("Drive")
{
    auto config = context->getConfig();
    front_left_module = std::make_shared<MK4SwerveModule>(
        config.drive_config.front_left_drive_id,
        config.drive_config.front_left_rotate_id,
        config.drive_config.front_left_encoder_id,
        config.drive_config.front_left_encoder_offset,
        "Front Left"
        );
    front_right_module = std::make_shared<MK4SwerveModule>(
        config.drive_config.front_right_drive_id,
        config.drive_config.front_right_rotate_id,
        config.drive_config.front_right_encoder_id,
        config.drive_config.front_right_encoder_offset,
        "Front Right"
        );
    back_left_module = std::make_shared<MK4SwerveModule>(
        config.drive_config.back_left_drive_id,
        config.drive_config.back_left_rotate_id,
        config.drive_config.back_left_encoder_id,
        config.drive_config.back_left_encoder_offset,
        "Back Left"
        );
    back_right_module = std::make_shared<MK4SwerveModule>(
        config.drive_config.back_right_drive_id,
        config.drive_config.back_right_rotate_id,
        config.drive_config.back_right_encoder_id,
        config.drive_config.back_right_encoder_offset,
        "Back Right"
        );

    swerve_drive_coordinator = std::make_unique<SwerveModuleCordinator>(
        front_left_module,
        front_right_module,
        back_left_module,
        back_right_module,
        config.base_size_config.wheel_base,
        config.base_size_config.track_width
    );

    //Odometry setup
    kinematics = std::make_unique<frc::SwerveDriveKinematics<4>>(
        frc::Translation2d
        {
            units::meter_t{config.drive_config.swerve_drive_kinematics.front_left_position.x},
            units::meter_t{config.drive_config.swerve_drive_kinematics.front_left_position.y}
        },
        frc::Translation2d
        {
            units::meter_t{config.drive_config.swerve_drive_kinematics.front_right_position.x},
            units::meter_t{config.drive_config.swerve_drive_kinematics.front_right_position.y}
        },
        frc::Translation2d
        {
            units::meter_t{config.drive_config.swerve_drive_kinematics.back_left_position.x},
            units::meter_t{config.drive_config.swerve_drive_kinematics.back_left_position.y}
        },
        frc::Translation2d
        {
            units::meter_t{config.drive_config.swerve_drive_kinematics.back_right_position.x},
            units::meter_t{config.drive_config.swerve_drive_kinematics.back_right_position.y}
        }
    );

    odometry = std::make_unique<frc::SwerveDrivePoseEstimator<4>>(
        *(kinematics.get()),
        frc::Rotation2d{units::degree_t(0.0)},
        std::array<frc::SwerveModulePosition,4>{
            front_left_module->getModulePosition(),
            front_right_module->getModulePosition(),
            back_left_module->getModulePosition(),
            back_right_module->getModulePosition(),
        },
        frc::Pose2d
        {
            units::meter_t{0.0},
            units::meter_t{0.0},
            units::degree_t{0.0}
        }
    );

    sysid_drive_routine = std::make_unique<frc2::sysid::SysIdRoutine>(
        frc2::sysid::Config{std::nullopt, std::nullopt, std::nullopt,
                          std::nullopt},
        frc2::sysid::Mechanism
        {
            [this](units::volt_t drive_voltage)
            {
                front_left_module->setDriveVoltage(drive_voltage);
                back_left_module->setDriveVoltage(drive_voltage);
                front_right_module->setDriveVoltage(drive_voltage);
                back_right_module->setDriveVoltage(drive_voltage);
            },
            [this](frc::sysid::SysIdRoutineLog* log)
            {
                log->Motor("drive-front-left")
                    .voltage(front_left_module->getDriveVoltage())
                    .position(front_left_module->getDrivePosition())
                    .velocity(front_left_module->getDriveVelocity());
            },
            this
        }
    );

    sysid_rotate_routine = std::make_unique<frc2::sysid::SysIdRoutine>(
        frc2::sysid::Config{frc2::sysid::ramp_rate_t{0.5}, units::volt_t{4.5}, std::nullopt,
                          std::nullopt},
        frc2::sysid::Mechanism
        {
            [this](units::volt_t drive_voltage)
            {
                front_left_module->setRotateVoltage(drive_voltage);
            },
            [this](frc::sysid::SysIdRoutineLog* log)
            {
                log->Motor("drive-front-left")
                    .voltage(front_left_module->getRotateVoltage())
                    .position(front_left_module->getRotatePosition())
                    .velocity(front_left_module->getRotateVelocity());
            },
            this
        }
    );

    q_f_command = std::make_unique<frc2::CommandPtr>(sysid_drive_routine->Quasistatic(frc2::sysid::Direction::kForward));
    q_r_command = std::make_unique<frc2::CommandPtr>(sysid_drive_routine->Quasistatic(frc2::sysid::Direction::kReverse));
    d_f_command = std::make_unique<frc2::CommandPtr>(sysid_drive_routine->Dynamic(frc2::sysid::Direction::kForward));
    d_r_command = std::make_unique<frc2::CommandPtr>(sysid_drive_routine->Dynamic(frc2::sysid::Direction::kReverse));

    //Smart dashboard
    frc::SmartDashboard::SetDefaultNumber(DriveSystem::SDB_DRIVE_ANGLE_KEY, 0.0);
    frc::SmartDashboard::SetDefaultNumber(DriveSystem::SDB_DRIVE_MAGNITUDE_KEY, 0.0);
    frc::SmartDashboard::SetDefaultNumber(DriveSystem::SDB_TURN_MAGNITUDE_KEY, 0.0);
    
    frc::SmartDashboard::SetDefaultNumber(DriveSystem::SDB_FLA_KEY, 0.0);
    frc::SmartDashboard::SetDefaultNumber(DriveSystem::SDB_FRA_KEY, 0.0);
    frc::SmartDashboard::SetDefaultNumber(DriveSystem::SDB_BLA_KEY, 0.0);
    frc::SmartDashboard::SetDefaultNumber(DriveSystem::SDB_BRA_KEY, 0.0);
    frc::SmartDashboard::SetDefaultNumber("FalconEncoderFL", 0.0);
    frc::SmartDashboard::SetDefaultNumber("FalconEncoderFR", 0.0);
    frc::SmartDashboard::SetDefaultNumber("FalconEncoderBL", 0.0);
    frc::SmartDashboard::SetDefaultNumber("FalconEncoderBR", 0.0);
    frc::SmartDashboard::SetDefaultNumber("SwerveOffset", 0.0);
    frc::SmartDashboard::SetDefaultNumber("encoder", 0.0);
    frc::SmartDashboard::SetDefaultNumber(DriveSystem::TARGET_POSITION_X_KEY, 0.0);
    frc::SmartDashboard::SetDefaultNumber(DriveSystem::TARGET_POSITION_Y_KEY, 0.0);
    frc::SmartDashboard::SetDefaultNumber(DriveSystem::TARGET_POSITION_ANGLE_KEY, 0.0);
    frc::SmartDashboard::SetDefaultNumber(DriveSystem::CURRENT_POSITION_X_KEY, 0.0);
    frc::SmartDashboard::SetDefaultNumber(DriveSystem::CURRENT_POSITION_Y_KEY, 0.0);
    frc::SmartDashboard::SetDefaultNumber(DriveSystem::CURRENT_POSITION_ANGLE_KEY, 0.0);
    frc::SmartDashboard::SetDefaultNumber(DriveSystem::MAX_DRIVE_SPEED_KEY, 0.0);
    frc::SmartDashboard::SetDefaultNumber(DriveSystem::DRIVE_TO_SPEED_KEY, 0.0);
    frc::SmartDashboard::SetDefaultNumber(DriveSystem::DRIVE_TO_ANGLE_KEY, 0.0);
    frc::SmartDashboard::SetDefaultNumber("Yaw", 0.0);
    frc::SmartDashboard::SetDefaultNumber("Test Voltage", 0.0);
    swerve_drive_coordinator->setFieldOriented(true);
}

void DriveSystem::updateAnglesSMD()
{
    frc::SmartDashboard::PutNumber("FalconEncoderFR", front_right_module->getRawAngle());
    frc::SmartDashboard::PutNumber("FalconEncoderBR", back_right_module->getRawAngle());
    frc::SmartDashboard::PutNumber("FalconEncoderFL", front_left_module->getRawAngle());
    frc::SmartDashboard::PutNumber("FalconEncoderBL", back_left_module->getRawAngle());
    frc::SmartDashboard::PutNumber("Yaw", context->getPigeon().GetYaw().GetValue().value());
}

void DriveSystem::Periodic()
{
    //swerve_drive_coordinator->setDrive(90.0,1.0,0.0);
    current_pose = updateOdometry();
    swerve_drive_coordinator->setFieldAngle(context->getPigeon().GetYaw().GetValue().value() - context->startAngle);
    switch(drive_mode)
    {
        case DriveMode::MANUAL:
        {
            //Nothing, drive direction is set by robot class
        }break;
        case DriveMode::DRIVE_TO:
        {
            doDriveTo();
        }break;
        case DriveMode::STOP:
        {
            doStop();
        }break;
        case DriveMode::TURN_TO:
        {
            doTurnTo();
        }break;
        case DriveMode::SYS_ID:
        {
            doSysId();
        }break;
        case DriveMode::AUTO_AIM:
        {
            doAutoAim();
        }break;
        case DriveMode::RING_LOCK:
        {
            doRingLock();
        }
    }

    if(drive_mode != DriveMode::SYS_ID)
    {
        swerve_drive_coordinator->update();
    }
    frc::SmartDashboard::PutNumber(DriveSystem::TARGET_POSITION_X_KEY, target_pose.X().value());
    frc::SmartDashboard::PutNumber(DriveSystem::TARGET_POSITION_Y_KEY, target_pose.Y().value());
    frc::SmartDashboard::PutNumber(DriveSystem::TARGET_POSITION_ANGLE_KEY, target_pose.Rotation().Degrees().value());
    frc::SmartDashboard::PutNumber(DriveSystem::CURRENT_POSITION_X_KEY, current_pose.X().value());
    frc::SmartDashboard::PutNumber(DriveSystem::CURRENT_POSITION_Y_KEY,  current_pose.Y().value());
    frc::SmartDashboard::PutNumber(DriveSystem::CURRENT_POSITION_ANGLE_KEY, current_pose.Rotation().Degrees().value());

    frc::SmartDashboard::PutNumber(DriveSystem::MAX_DRIVE_SPEED_KEY, maxDriveSpeed);
}

void DriveSystem::loadFromConfig()
{
    auto config = context->getConfig();
    PID new_pid(
        config.drive_config.swerve_rotate_pid.p,
        config.drive_config.swerve_rotate_pid.i,
        config.drive_config.swerve_rotate_pid.d
    );
    front_left_module->setAnglePID(config.drive_config.swerve_rotate_pid, config.drive_config.swerve_rotate_trapazoidal);
    front_right_module->setAnglePID(config.drive_config.swerve_rotate_pid, config.drive_config.swerve_rotate_trapazoidal);
    back_left_module->setAnglePID(config.drive_config.swerve_rotate_pid, config.drive_config.swerve_rotate_trapazoidal);
    back_right_module->setAnglePID(config.drive_config.swerve_rotate_pid, config.drive_config.swerve_rotate_trapazoidal);
    front_left_module->setAngleFeedForward(config.drive_config.swerve_rotate_feed_forward);
    front_right_module->setAngleFeedForward(config.drive_config.swerve_rotate_feed_forward);
    back_left_module->setAngleFeedForward(config.drive_config.swerve_rotate_feed_forward);
    back_right_module->setAngleFeedForward(config.drive_config.swerve_rotate_feed_forward);

    front_left_module->setDriveFeedForward(config.drive_config.swerve_drive_feed_forward);
    front_right_module->setDriveFeedForward(config.drive_config.swerve_drive_feed_forward);
    back_left_module->setDriveFeedForward(config.drive_config.swerve_drive_feed_forward);
    back_right_module->setDriveFeedForward(config.drive_config.swerve_drive_feed_forward);
    front_left_module->setDrivePID(config.drive_config.swerve_drive_pid);
    back_left_module->setDrivePID(config.drive_config.swerve_drive_pid);
    front_right_module->setDrivePID(config.drive_config.swerve_drive_pid);
    back_right_module->setDrivePID(config.drive_config.swerve_drive_pid);

}

void DriveSystem::reset()
{
    front_left_module->resetError();
    front_right_module->resetError();
    back_left_module->resetError();
    back_right_module->resetError();
    angle_correction_pid.resetError();
    target_angle = 0.0;
}

void DriveSystem::setDrive(double translate_x, double translate_y, double turn)
{
    double stick_angle = Utility::get180Range(Utility::toDegrees(std::atan2(-translate_y, translate_x)) - 90.0);
    double stick_magnitude = std::sqrt(translate_x*translate_x + translate_y*translate_y);//std::pow is slow relative to multiplication, just multiply by itself to square
    double current_angle = getRobotAngle();
    if(stick_magnitude > translate_magnitude_deadzone)
    {
        if(std::abs(turn) > turn_magnitude_deadzone)
        {
            target_angle = current_angle;
            angle_correction_pid.resetError();
        }
        else
        {
            turn = angle_correction_pid.calculatePID(target_angle, current_angle);
        }
        
        swerve_drive_coordinator->setDrive(stick_angle, stick_magnitude, turn);
    }
    else if(std::abs(turn) > turn_magnitude_deadzone)
    {
        swerve_drive_coordinator->setRotate(turn);
        target_angle = current_angle;
    }
    else
    {
        swerve_drive_coordinator->stopDrive();
    }
    frc::SmartDashboard::PutNumber(DriveSystem::SDB_DRIVE_ANGLE_KEY, stick_angle);
    frc::SmartDashboard::PutNumber(DriveSystem::SDB_DRIVE_MAGNITUDE_KEY, stick_magnitude);
    frc::SmartDashboard::PutNumber(DriveSystem::SDB_TURN_MAGNITUDE_KEY, turn);

    frc::SmartDashboard::PutNumber(DriveSystem::SDB_FLA_KEY, front_left_module->getTargetAngle());
    frc::SmartDashboard::PutNumber(DriveSystem::SDB_FRA_KEY, front_right_module->getTargetAngle());
    frc::SmartDashboard::PutNumber(DriveSystem::SDB_BLA_KEY, back_left_module->getTargetAngle());
    frc::SmartDashboard::PutNumber(DriveSystem::SDB_BRA_KEY, back_right_module->getTargetAngle());

    frc::SmartDashboard::UpdateValues();
}

void DriveSystem::startLogging()
{
    logger = std::unique_ptr<CSVLogger<5>>{new CSVLogger<5>{{"time", "front left", "front right", "back left", "back right"}, "/home/lvuser/drivelog.csv"}};
    std::cout << "starting logging" << std::endl;
    log_start_time = std::chrono::steady_clock::now();
}

void DriveSystem::endLogging()
{
    logger->flush();
    logger.reset();
}

void DriveSystem::addToLuaState(Keeko::ThreadedScript& script)
{
    luabridge::getGlobalNamespace(script)
        .beginClass<DriveSystem> ("DriveSystem")
            .addFunction("setDrive", &DriveSystem::setDrive)
            .addFunction("getDriveMode",&DriveSystem::getCurrentDriveModeAsString)
            .addFunction("stop", &DriveSystem::stop)
            .addFunction("isAtTarget", &DriveSystem::isAtTarget)
            .addFunction("driveTo", &DriveSystem::driveToLua)
            .addFunction("setMaxDriveSpeed", &DriveSystem::setMaxDriveSpeed)
            .addFunction("getMaxDriveSpeed", &DriveSystem::getMaxDriveSpeed)
            .addFunction("getDistanceFromTarget", &DriveSystem::getDistanceFromTarget)
            .addFunction("getAngleFromTarget", &DriveSystem::getAngleFromTarget)
            .addFunction("setMinDrivePercentage", &DriveSystem::setMinDrivePercentage)
            .addFunction("setOdometry", &DriveSystem::setOdometry)
            .addFunction("turnTo", &DriveSystem::turnToLua)
            .addFunction("pointTo", &DriveSystem::pointTo)
            .addFunction("getRobotAngle", &DriveSystem::getRobotAngle)
            .addFunction("autoAim", &DriveSystem::autoAim)
            .addFunction("ringLock", &DriveSystem::ringLock)
            .addFunction("getRobotX", &DriveSystem::getRobotX)
            .addFunction("ringLockedOn", &DriveSystem::ringLockedOn)
            .addFunction("getTime", &DriveSystem::getTime)
            .addFunction("distanceToSpeaker", &DriveSystem::getDistanceToSpeaker)
            .addFunction("speakerLockedOn", &DriveSystem::speakerLockedOn)
            .addFunction("getRobotAngle", &DriveSystem::getRobotAngle)
        .endClass();
    script.addInstance("drive_system", this);
}

double DriveSystem::getRobotAngle()
{
    return context->getPigeon().GetYaw().GetValue().value() - context->startAngle;
}

double DriveSystem::getRobotX()
{
    return current_pose.X().value();
}

units::degree_t DriveSystem::getRobotAngleInDegrees()
{
    return units::degree_t{getRobotAngle()};
}

frc::Pose2d DriveSystem::updateOdometry()
{
    auto mp = back_right_module->getModulePosition();
    frc::SmartDashboard::PutNumber("encoder", mp.distance.value());
    context->getVision().applyVisionData(*odometry);
    return odometry->Update(
        getRobotAngleInDegrees(),
        std::array<frc::SwerveModulePosition,4>{
            front_left_module->getModulePosition(),
            front_right_module->getModulePosition(),
            back_left_module->getModulePosition(),
            back_right_module->getModulePosition(),
        }
    );
}

std::string DriveSystem::getCurrentDriveModeAsString()
{
    // std::unique_lock<std::mutex> lock{mutex};
    return DriveSystem::driveModeToString(drive_mode);
}

void DriveSystem::setDriveMode(DriveMode mode)
{
    // std::unique_lock<std::mutex> lock{mutex};
    drive_mode = mode;
}

void DriveSystem::setSysIdMode(SysIdMode mode)
{
    sys_id_mode = mode;
}

void DriveSystem::stop()
{
    // std::unique_lock<std::mutex> lock{mutex};
    drive_mode = DriveMode::STOP;
    at_target = true;
}

void DriveSystem::doStop()
{
    swerve_drive_coordinator->stopDrive();
}

void DriveSystem::driveTo(frc::Pose2d pose)
{
    // std::unique_lock<std::mutex> lock{mutex};
    drive_mode = DriveMode::DRIVE_TO;
    at_target = false;
    target_pose = pose;
}

void DriveSystem::driveToLua(double x, double y, double angle)
{
    frc::Pose2d pose{
        units::meter_t{x},
        units::meter_t{y},
        frc::Rotation2d{units::degree_t{angle}}
    };
    driveTo(pose);
}

bool DriveSystem::isAtTarget()
{
    // std::unique_lock<std::mutex> lock{mutex};
    return at_target;
}

void DriveSystem::setMaxDriveSpeed(double speed)
{
    // std::unique_lock<std::mutex> lock{mutex};
    front_left_module->setMaxSpeed(speed);
    front_right_module->setMaxSpeed(speed);
    back_left_module->setMaxSpeed(speed);
    back_right_module->setMaxSpeed(speed);
    maxDriveSpeed = speed;
}

double DriveSystem::getMaxDriveSpeed()
{
    return maxDriveSpeed;
}

double DriveSystem::getDistanceFromTarget()
{
    auto delta_pose = target_pose - current_pose;
    double x = delta_pose.X().value();
    double y = delta_pose.Y().value();
    return std::sqrt(x*x + y*y);
}

double DriveSystem::getAngleFromTarget()
{
    auto delta_pose = target_pose - current_pose;
    return Utility::get180Range(Utility::toDegrees(std::atan2(delta_pose.Y().value(), delta_pose.X().value())));
}

void DriveSystem::doDriveTo()
{
    swerve_drive_coordinator->setFieldOriented(true);
    double angle = getAngleFromTarget();
    double distance = getDistanceFromTarget();
    double speed = std::min(1.0, distance*drive_to_proportional);
    double current_angle = getRobotAngle();
    double angle_offset = Utility::get180Range(target_pose.Rotation().Degrees().value() - current_angle);
    double turn = angle_correction_pid.calculatePID(angle_offset, 0);
    turn = Utility::minMax(-0.5, 0.5, turn);
    speed = std::max(speed, minDrivePercentage);
    swerve_drive_coordinator->setDrive(angle + current_angle, speed, turn);
    frc::SmartDashboard::PutNumber(DRIVE_TO_SPEED_KEY, speed);
    frc::SmartDashboard::PutNumber(DriveSystem::DRIVE_TO_ANGLE_KEY, angle);
}

void DriveSystem::doTurnTo()
{
    double current_angle = getRobotAngle();
    double angle_offset = Utility::get180Range(target_angle - current_angle);
    double power = turn_to_pid.Calculate(angle_offset, 0.0);
    swerve_drive_coordinator->setFieldOriented(true);
    swerve_drive_coordinator->setRotate(power);
    if(std::abs(angle_offset) < turn_to_tolerance)
    {
        at_target = true;
    }
    else
    {
        at_target = false;
    }
}

void DriveSystem::doSysId()
{
    front_left_module->setRotateVoltage(units::volt_t{0.0});
    back_left_module->setRotateVoltage(units::volt_t{0.0});
    front_right_module->setRotateVoltage(units::volt_t{0.0});
    back_right_module->setRotateVoltage(units::volt_t{0.0});
    switch(sys_id_mode)
    {
        case SysIdMode::IDLE:
        {
            front_left_module->setDriveVoltage(units::volt_t{0.0});
            back_left_module->setDriveVoltage(units::volt_t{0.0});
            front_right_module->setDriveVoltage(units::volt_t{0.0});
            back_right_module->setDriveVoltage(units::volt_t{0.0});
            q_f_command->Cancel();
            q_r_command->Cancel();
            d_f_command->Cancel();
            d_r_command->Cancel();

        }break;
        case SysIdMode::QUASISTATIC_FORWARD:
        {
            if(!q_f_command->IsScheduled())
            {
                q_f_command->Schedule();
            }
        }break;
        case SysIdMode::QUASISTATIC_REVERSE:
        {
            if(!q_r_command->IsScheduled())
            {
                q_r_command->Schedule();
            }
        }break;
        case SysIdMode::DYNAMIC_FORWARD:
        {
            if(!d_f_command->IsScheduled())
            {
                d_f_command->Schedule();
            }
        }break;
        case SysIdMode::DYNAMIC_REVERSE:
        {
            if(!d_r_command->IsScheduled())
            {
                d_r_command->Schedule();
            }
        }break;

    }
}

void DriveSystem::setMinDrivePercentage(double percentage)
{
    minDrivePercentage = percentage;
}

void DriveSystem::setToBrake()
{
    front_left_module->setToBrake();
    front_right_module->setToBrake();
    back_left_module->setToBrake();
    back_right_module->setToBrake();
}

void DriveSystem::setToCoast()
{
    front_left_module->setToCoast();
    front_right_module->setToCoast();
    back_left_module->setToCoast();
    back_right_module->setToCoast();
}

void DriveSystem::setOdometry(double x, double y, double angle)
{
    context->startAngle =  context->getPigeon().GetYaw().GetValue().value() - angle;
    odometry->ResetPosition(frc::Rotation2d(getRobotAngleInDegrees()),
    std::array<frc::SwerveModulePosition,4>{
            front_left_module->getModulePosition(),
            front_right_module->getModulePosition(),
            back_left_module->getModulePosition(),
            back_right_module->getModulePosition(),
        },
        frc::Pose2d(units::meter_t{x}, units::meter_t{y}, units::degree_t{angle})
    );
}

void DriveSystem::pointTo(double x, double y)
{
    target_angle = getAngleToPoint(x, y);
    at_target = false;
    drive_mode = DriveMode::TURN_TO;
}

double DriveSystem::getAngleToPoint(double x, double y)
{
    double current_x = current_pose.X().value();
    double current_y = current_pose.Y().value();

    x-= current_x;
    y-= current_y;
    return Utility::toDegrees(std::atan2(y, x));
}

void DriveSystem::turnToLua(double angle)
{
    // std::unique_lock<std::mutex> lock{mutex};
    drive_mode = DriveMode::TURN_TO;
    at_target = false;
    target_angle = angle;
}

double DriveSystem::getRobotAngleLua()
{
    return 0.0;
}

frc2::CommandPtr DriveSystem::getDriveSysIdCommand(SysIdMode mode)
{
    switch(mode)
    {
        case SysIdMode::QUASISTATIC_FORWARD: return sysid_drive_routine->Quasistatic(frc2::sysid::Direction::kForward);
        case SysIdMode::QUASISTATIC_REVERSE: return sysid_drive_routine->Quasistatic(frc2::sysid::Direction::kReverse);
        case SysIdMode::DYNAMIC_FORWARD: return sysid_drive_routine->Dynamic(frc2::sysid::Direction::kForward);
        case SysIdMode::DYNAMIC_REVERSE: return sysid_drive_routine->Dynamic(frc2::sysid::Direction::kReverse);
        default: return sysid_drive_routine->Quasistatic(frc2::sysid::Direction::kForward); //Something went wrong, 
    }
}

frc2::CommandPtr DriveSystem::getRotateSysIdCommand(SysIdMode mode)
{
    switch(mode)
    {
        case SysIdMode::QUASISTATIC_FORWARD: return sysid_rotate_routine->Quasistatic(frc2::sysid::Direction::kForward);
        case SysIdMode::QUASISTATIC_REVERSE: return sysid_rotate_routine->Quasistatic(frc2::sysid::Direction::kReverse);
        case SysIdMode::DYNAMIC_FORWARD: return sysid_rotate_routine->Dynamic(frc2::sysid::Direction::kForward);
        case SysIdMode::DYNAMIC_REVERSE: return sysid_rotate_routine->Dynamic(frc2::sysid::Direction::kReverse);
        default: return sysid_rotate_routine->Quasistatic(frc2::sysid::Direction::kForward); //Something went wrong, 
    }
}

void DriveSystem::autoAim(double controller_x, double controller_y)
{
    drive_mode = DriveMode::AUTO_AIM;
    user_x = controller_x;
    user_y = controller_y;
}

void DriveSystem::ringLock(double forward)
{
    drive_mode = DriveMode::RING_LOCK;
    ring_lock_forward = -forward;
}

//HERE: the auto aim mode
void DriveSystem::doAutoAim()
{
    swerve_drive_coordinator->setFieldOriented(true);
    auto target_info = context->getVision().getSpeakerTargetOffset();//May need to make negative
    double target_angle = 0.0;
    double target_distance = 0.0;
    double turn_power = 0.0;
    if(target_info.has_value())
    {
        // //try to point in general direction of speaker when not in view
        // //not super important, just a convenience for drivers
        // target_angle = getAngleToPoint(speaker_pose[0], speaker_pose[1]);

        target_angle = target_info.value().offset;
        target_distance = target_info.value().distance;
    }
    turn_power = auto_aim_pid.Calculate(target_angle, 0.0);
    double drive_angle = Utility::get180Range(Utility::toDegrees(std::atan2(-user_y, user_x)) - 90.0);
    double drive_magnitude = std::sqrt(user_x*user_x + user_y*user_y);
    if(drive_magnitude < 0.1)
    {
        swerve_drive_coordinator->setRotate(-turn_power);
        turn_power = Utility::minMax(-2.0, 2.0, turn_power);
    }
    else
    {
        turn_power = Utility::minMax(-1.0, 1.0, turn_power*5);
        std::cout << turn_power << std::endl;
        swerve_drive_coordinator->setDrive(drive_angle, drive_magnitude, -turn_power);
    }
}

void DriveSystem::doRingLock()
{
    swerve_drive_coordinator->setFieldOriented(false);
    auto target_angle = context->getVision().getRingTargetOffset();
    double turn_power = 0.0;
    if(target_angle.has_value())
    {
        turn_power = -auto_aim_pid.Calculate(target_angle.value(), 0.0) * 2.0;
    }
    if(std::abs(ring_lock_forward) > 0.05)
    {
        swerve_drive_coordinator->setDrive(0.0, ring_lock_forward, turn_power);
    }
    else
    {
        swerve_drive_coordinator->setRotate(turn_power);
    }
}

void DriveSystem::setFieldOriented(bool is_field)
{
    swerve_drive_coordinator->setFieldOriented(is_field);
}

bool DriveSystem::ringLockedOn()
{
    auto target_angle = context->getVision().getRingTargetOffset();
    if(target_angle.has_value() && std::abs(target_angle.value()) <= LOCK_ON_THRESHOLD)
    {
        return true;
    }
    return false;
}

double DriveSystem::getTime()
{
    return frc::Timer::GetFPGATimestamp().value();
}

bool DriveSystem::speakerLockedOn()
{
    auto target_angle = context->getVision().getSpeakerTargetOffset();
    if(target_angle.has_value() && std::abs(target_angle.value().offset) <= LOCK_ON_THRESHOLD)
    {
        return true;
    }
    return false;
}

void DriveSystem::setSpeakerPose(std::string color)
{
    if(color == "red")
    {
        speaker_pose = {0.0, -1.4478};
    }
    else
    {
        speaker_pose = {0.0, 1.4478};
    }
}

std::optional<double> DriveSystem::getDistanceToSpeaker()
{
    std::optional<double> distance;
    if(context->getVision().speakerInView())
    {
        double x = current_pose.X().value();
        double y = current_pose.Y().value();
        x -= speaker_pose[0];
        y -= speaker_pose[1];

        return std::sqrt(x*x + y*y);
    }
    return std::nullopt;
}
