#include "RobotConfig.h"

void from_json(const nlohmann::json& j, PIDConfig& p)
{
    p.p = j["p"];
    p.i = j["i"];
    p.d = j["d"];
}

void from_json(const nlohmann::json& j, FeedForwardConfig& ff)
{
    ff.k_s = j["kS"];
    ff.k_v = j["kV"];
    ff.k_a = j["kA"];
    ff.k_g = j.value<double>("kG", 0.0);

}

void from_json(const nlohmann::json& j, TrapazoidalProfileConfig& tp)
{
    tp.max_velocity = j["velocity"];
    tp.max_acceleration = j["acceleration"];
}

void from_json(const nlohmann::json& j, Position2DConfig& p2d)
{
    p2d.x = j[Position2DConfig::X_KEY];
    p2d.y = j[Position2DConfig::Y_KEY];
}

void from_json(const nlohmann::json& j, SwerveKinematicsConfig& sk)
{
    sk.front_left_position = j[SwerveKinematicsConfig::FRONT_LEFT_POSITION_KEY];
    sk.front_right_position = j[SwerveKinematicsConfig::FRONT_RIGHT_POSITION_KEY];
    sk.back_left_position = j[SwerveKinematicsConfig::BACK_LEFT_POSITION_KEY];
    sk.back_right_position = j[SwerveKinematicsConfig::BACK_RIGHT_POSITION_KEY];
}

void from_json(const nlohmann::json& j, IntakeConfig& ic)
{
    ic.intake_motor_id = j[IntakeConfig::INTAKE_MOTOR_ID_KEY];
    ic.linebreak_id = j[IntakeConfig::INTAKE_MOTOR_ID_KEY];
    ic.servo_pwm_id = j[IntakeConfig::SERVO_PWM_ID_KEY];
}

void from_json(const nlohmann::json& j, DriveConfig& d)
{
    //Motion profiles
    d.swerve_rotate_pid = j[DriveConfig::SWERVE_ROTATE_PID_KEY];
    d.swerve_rotate_feed_forward = j[DriveConfig::SWERVE_ROTATE_FF_KEY];
    d.swerve_rotate_trapazoidal = j[DriveConfig::SWERVE_ROTATE_TRAPAZOIDAL_KEY];
    d.swerve_drive_feed_forward = j[DriveConfig::SWERVE_DRIVE_FF_KEY];
    d.swerve_drive_pid = j[DriveConfig::SWERVE_DRIVE_PID_KEY];

    //CAN IDs
    d.front_left_drive_id = j[DriveConfig::FRONT_LEFT_DRIVE_ID_KEY];
    d.front_left_rotate_id = j[DriveConfig::FRONT_LEFT_ROTATE_ID_KEY];
    d.front_left_encoder_id = j[DriveConfig::FRONT_LEFT_ENCODER_ID_KEY];

    d.back_left_drive_id = j[DriveConfig::BACK_LEFT_DRIVE_ID_KEY];
    d.back_left_rotate_id = j[DriveConfig::BACK_LEFT_ROTATE_ID_KEY];
    d.back_left_encoder_id = j[DriveConfig::BACK_LEFT_ENCODER_ID_KEY];

    d.front_right_drive_id = j[DriveConfig::FRONT_RIGHT_DRIVE_ID_KEY];
    d.front_right_rotate_id = j[DriveConfig::FRONT_RIGHT_ROTATE_ID_KEY];
    d.front_right_encoder_id = j[DriveConfig::FRONT_RIGHT_ENCODER_ID_KEY];

    d.back_right_drive_id = j[DriveConfig::BACK_RIGHT_DRIVE_ID_KEY];
    d.back_right_rotate_id = j[DriveConfig::BACK_RIGHT_ROTATE_ID_KEY];
    d.back_right_encoder_id = j[DriveConfig::BACK_RIGHT_ENCODER_ID_KEY];

    //Sensor offsets
    d.front_left_encoder_offset = j[DriveConfig::FRONT_LEFT_ENCODER_OFFSET_KEY];
    d.back_left_encoder_offset = j[DriveConfig::BACK_LEFT_ENCODER_OFFSET_KEY];
    d.front_right_encoder_offset = j[DriveConfig::FRONT_RIGHT_ENCODER_OFFSET_KEY];
    d.back_right_encoder_offset = j[DriveConfig::BACK_RIGHT_ENCODER_OFFSET_KEY];

    d.swerve_drive_kinematics = j[DriveConfig::SWERVE_DRIVE_KINEMATICS_KEY];
}

void from_json(const nlohmann::json& j, BaseSizeConfig& r)
{
    r.track_width = j[BaseSizeConfig::TRACK_WIDTH_KEY];
    r.wheel_base = j[BaseSizeConfig::WHEEL_BASE_KEY];
}

void from_json(const nlohmann::json& j, RobotConfig& r)
{
    r.drive_config = j[RobotConfig::DRIVE_CONFIG_KEY];
    r.climb_config = j[RobotConfig::CLIMB_CONFIG_KEY];
    r.intake_config = j[RobotConfig::INTAKE_CONFIG_KEY];
    r.shooter_config = j[RobotConfig::SHOOTER_CONFIG_KEY];
    r.base_size_config = j[RobotConfig::BASE_SIZE_CONFIG_ID];
}

void from_json(const nlohmann::json& j, ClimbConfig& cc)
{
    cc.arm_left_id = j[ClimbConfig::ARM_LEFT_ID_KEY];
    cc.arm_right_id = j[ClimbConfig::ARM_RIGHT_ID_KEY];
    cc.arm_encoder_id = j[ClimbConfig::ARM_ENCODER_ID_KEY];
    cc.elevator_left_id = j[ClimbConfig::ELEVATOR_LEFT_ID_KEY];
    cc.elevator_right_id = j[ClimbConfig::ELEVATOR_RIGHT_ID_KEY];
    cc.elevator_encoder_id = j[ClimbConfig::ELEVATOR_ENCODER_ID_KEY];
    cc.arm_encoder_offset = j[ClimbConfig::ARM_ENCODER_OFFSET_KEY];
}

void from_json(const nlohmann::json& j, ShooterConfig& sc)
{
    sc.shooter_top_id = j[ShooterConfig::SHOOTER_TOP_ID_KEY];
    sc.shooter_bottom_id = j[ShooterConfig::SHOOTER_BOTTOM_ID_KEY];
    sc.shooter_feedforward = j[ShooterConfig::SHOOTER_FEEDFORWARD_KEY];
    sc.shooter_pid = j[ShooterConfig::SHOOTER_PID_KEY];
    sc.arm_feedforward = j.value<FeedForwardConfig>(ShooterConfig::ARM_FEEDFORWARD_KEY, FeedForwardConfig{});
    sc.arm_pid = j.value<PIDConfig>(ShooterConfig::ARM_PID_KEY, PIDConfig{});
}



