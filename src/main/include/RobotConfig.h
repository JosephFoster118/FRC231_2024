#pragma once
#include <nlohmann/json.hpp>

struct PIDConfig
{
    float p{0};
    float i{0};
    float d{0};
    friend void from_json(const nlohmann::json& j, PIDConfig& p);

};

struct FeedForwardConfig
{
    double k_s{0};
    double k_v{0};
    double k_a{0};
    double k_g{0};
    friend void from_json(const nlohmann::json& j, FeedForwardConfig& p);
};

struct TrapazoidalProfileConfig
{
    double max_velocity{0};
    double max_acceleration{0};
    friend void from_json(const nlohmann::json& j, TrapazoidalProfileConfig& tp);
};

struct Position2DConfig
{
    double x{0};
    double y{0};
    friend void from_json(const nlohmann::json& j, Position2DConfig& p2d);
    inline static const std::string X_KEY{"x"};
    inline static const std::string Y_KEY{"y"};
};

struct SwerveKinematicsConfig
{
    Position2DConfig front_left_position;
    Position2DConfig front_right_position;
    Position2DConfig back_left_position;
    Position2DConfig back_right_position;
    friend void from_json(const nlohmann::json& j, SwerveKinematicsConfig& sk);
    inline static const std::string FRONT_LEFT_POSITION_KEY{"front left"};
    inline static const std::string FRONT_RIGHT_POSITION_KEY{"front right"};
    inline static const std::string BACK_LEFT_POSITION_KEY{"back left"};
    inline static const std::string BACK_RIGHT_POSITION_KEY{"back right"};
};

struct DriveConfig
{
    //Motion profiles
    PIDConfig swerve_rotate_pid;
    FeedForwardConfig swerve_rotate_feed_forward;
    TrapazoidalProfileConfig swerve_rotate_trapazoidal;
    FeedForwardConfig swerve_drive_feed_forward;
    PIDConfig swerve_drive_pid;

    //CAN IDs
    uint8_t front_left_drive_id;
    uint8_t front_left_rotate_id;
    uint8_t front_left_encoder_id;
    uint8_t front_right_drive_id;
    uint8_t front_right_rotate_id;
    uint8_t front_right_encoder_id;
    uint8_t back_left_drive_id;
    uint8_t back_left_rotate_id;
    uint8_t back_left_encoder_id;
    uint8_t back_right_drive_id;
    uint8_t back_right_rotate_id;
    uint8_t back_right_encoder_id;

    //Sensor offsets
    float front_left_encoder_offset;
    float back_left_encoder_offset;
    float front_right_encoder_offset;
    float back_right_encoder_offset;

    //Odometry
    SwerveKinematicsConfig swerve_drive_kinematics;

    friend void from_json(const nlohmann::json& j, DriveConfig& d);
    //constants
    inline static const std::string SWERVE_ROTATE_PID_KEY{"swerve rotate pid"};
    inline static const std::string SWERVE_ROTATE_FF_KEY{"swerve rotate feedforward"};
    inline static const std::string SWERVE_ROTATE_TRAPAZOIDAL_KEY{"swerve rotate trapazoidal"};
    inline static const std::string SWERVE_DRIVE_FF_KEY{"swerve drive feedforward"};
    inline static const std::string SWERVE_DRIVE_PID_KEY{"swerve drive pid"};

    inline static const std::string FRONT_LEFT_DRIVE_ID_KEY{"front left drive"};
    inline static const std::string FRONT_LEFT_ROTATE_ID_KEY{"front left rotate"};
    inline static const std::string FRONT_LEFT_ENCODER_ID_KEY{"front left encoder"};
    inline static const std::string FRONT_RIGHT_DRIVE_ID_KEY{"front right drive"};
    inline static const std::string FRONT_RIGHT_ROTATE_ID_KEY{"front right rotate"};
    inline static const std::string FRONT_RIGHT_ENCODER_ID_KEY{"front right encoder"};
    inline static const std::string BACK_RIGHT_DRIVE_ID_KEY{"back right drive"};
    inline static const std::string BACK_RIGHT_ROTATE_ID_KEY{"back right rotate"};
    inline static const std::string BACK_RIGHT_ENCODER_ID_KEY{"back right encoder"};
    inline static const std::string BACK_LEFT_DRIVE_ID_KEY{"back left drive"};
    inline static const std::string BACK_LEFT_ROTATE_ID_KEY{"back left rotate"};
    inline static const std::string BACK_LEFT_ENCODER_ID_KEY{"back left encoder"};

    inline static const std::string FRONT_LEFT_ENCODER_OFFSET_KEY{"front left encoder offset"};
    inline static const std::string BACK_LEFT_ENCODER_OFFSET_KEY{"back left encoder offset"};
    inline static const std::string FRONT_RIGHT_ENCODER_OFFSET_KEY{"front right encoder offset"};
    inline static const std::string BACK_RIGHT_ENCODER_OFFSET_KEY{"back right encoder offset"};

    inline static const std::string SWERVE_DRIVE_KINEMATICS_KEY{"swerve kinematics"};
};

struct ClimbConfig
{
    uint8_t arm_left_id;
    uint8_t arm_right_id;
    uint8_t arm_encoder_id;
    uint8_t elevator_left_id;
    uint8_t elevator_right_id;
    uint8_t elevator_encoder_id;

    double arm_encoder_offset;

    inline static const std::string ARM_LEFT_ID_KEY{"arm left"};
    inline static const std::string ARM_RIGHT_ID_KEY{"arm right"};
    inline static const std::string ARM_ENCODER_ID_KEY{"arm encoder"};
    inline static const std::string ARM_ENCODER_OFFSET_KEY{"arm encoder offset"};
    inline static const std::string ELEVATOR_LEFT_ID_KEY{"elevator left"};
    inline static const std::string ELEVATOR_RIGHT_ID_KEY{"elevator right"};
    inline static const std::string ELEVATOR_ENCODER_ID_KEY{"elevator encoder"};

    friend void from_json(const nlohmann::json& j, ClimbConfig& cc);

};

struct IntakeConfig
{
    uint8_t intake_motor_id;
    uint8_t linebreak_id;
    uint8_t servo_pwm_id;

    inline static const std::string INTAKE_MOTOR_ID_KEY{"intake motor"};
    inline static const std::string LINEBREAK_ID_KEY{"linebreak"};
    inline static const std::string SERVO_PWM_ID_KEY{"servo"};

    friend void from_json(const nlohmann::json& j, IntakeConfig& ic);
};

struct ShooterConfig
{
    uint8_t shooter_top_id;
    uint8_t shooter_bottom_id;
    FeedForwardConfig shooter_feedforward;
    PIDConfig shooter_pid;
    FeedForwardConfig arm_feedforward;
    PIDConfig arm_pid;

    //arm presets


    inline static const std::string SHOOTER_TOP_ID_KEY{"shooter top"};
    inline static const std::string SHOOTER_BOTTOM_ID_KEY{"shooter bottom"};
    inline static const std::string SHOOTER_FEEDFORWARD_KEY{"shooter feedforward"};
    inline static const std::string SHOOTER_PID_KEY{"shooter pid"};
    inline static const std::string ARM_FEEDFORWARD_KEY{"arm feedforward"};
    inline static const std::string ARM_PID_KEY{"arm pid"};

    friend void from_json(const nlohmann::json& j, ShooterConfig& sc);
};

struct BaseSizeConfig
{
    double track_width;
    double wheel_base;

    inline static const std::string WHEEL_BASE_KEY{"wheel base"};
    inline static const std::string TRACK_WIDTH_KEY{"track width"};
};

struct RobotConfig
{
    DriveConfig drive_config;
    ClimbConfig climb_config;
    IntakeConfig intake_config;
    ShooterConfig shooter_config;

    inline static const std::string DRIVE_CONFIG_KEY{"drive"};
    inline static const std::string CLIMB_CONFIG_KEY{"climb"};
    inline static const std::string INTAKE_CONFIG_KEY{"intake"};
    inline static const std::string SHOOTER_CONFIG_KEY{"shooter"};

    BaseSizeConfig base_size_config;
    inline static const std::string BASE_SIZE_CONFIG_ID{"base size"};

    friend void from_json(const nlohmann::json& j, RobotConfig& r);
    
};



