#pragma once
#include "SwerveDriveCordinator.h"
#include "MK4SwerveModule.h"
#include "CSVLogger.h"

//Standard Lib
#include <memory>
#include <chrono>
#include <assert.h>     /* assert */
#include <sstream>
#include <mutex>

//Frc Lib
#include <frc2/command/Subsystem.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/controller/PIDController.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/DriverStation.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/Commands.h>


//Our Lib
#include "RobotContext.h"
#include "Vision.h"
#include "PID.h"
#include <Keeko/LuaException.h>
#include <Keeko/Scriptable.h>
#include <Keeko/ThreadedScript.h>
#include <Debounce.h>

//Third Party
#include "lua.h"
#include "lauxlib.h"
#include "lualib.h"
#include "LuaBridge.h"


//TODO: Add race condition prevention


class DriveSystem: public frc2::SubsystemBase
{
public:
    DriveSystem() = delete;
    DriveSystem(const DriveSystem&) = delete;//Removes the copy constructor
    DriveSystem(std::shared_ptr<RobotContext> context);

    //Constants and structures
    enum class DriveMode
    {
        MANUAL,
        DRIVE_TO,
        STOP,
        TURN_TO,
        SYS_ID,
        AUTO_AIM,
        RING_LOCK
    };


    enum class SysIdMode
    {
        QUASISTATIC_FORWARD,
        QUASISTATIC_REVERSE,
        DYNAMIC_FORWARD,
        DYNAMIC_REVERSE,
        IDLE
    };

    inline static std::string driveModeToString(DriveMode mode)
    {
        switch(mode)
        {
            case DriveMode::MANUAL: return "manual";
            case DriveMode::DRIVE_TO: return "drive to";
            case DriveMode::STOP: return "stop";
            case DriveMode::TURN_TO: return "turn to";
            case DriveMode::SYS_ID: return "sysid";
            case DriveMode::AUTO_AIM: return "auto_aim";
            case DriveMode::RING_LOCK: return "ring_lock";
        }
        return "unknown";
    }

    inline static const std::string SDB_DRIVE_ANGLE_KEY{"Drive Angle"};
    inline static const std::string SDB_DRIVE_MAGNITUDE_KEY{"Drive Magnitude"};
    inline static const std::string SDB_TURN_MAGNITUDE_KEY{"Turn Magnitude"};

    inline static const std::string SDB_FLA_KEY{"FLA"};
    inline static const std::string SDB_FRA_KEY{"FRA"};
    inline static const std::string SDB_BLA_KEY{"BLA"};
    inline static const std::string SDB_BRA_KEY{"BRA"};

    inline static const std::string TARGET_POSITION_X_KEY{"Target X"};
    inline static const std::string TARGET_POSITION_Y_KEY{"Target Y"};
    inline static const std::string TARGET_POSITION_ANGLE_KEY{"Target Angle"};
    inline static const std::string CURRENT_POSITION_X_KEY{"Current X"};
    inline static const std::string CURRENT_POSITION_Y_KEY{"Current Y"};
    inline static const std::string CURRENT_POSITION_ANGLE_KEY{"Current Angle"};
    inline static const std::string MAX_DRIVE_SPEED_KEY{"Max Drive Speed"};
    inline static const std::string DRIVE_TO_SPEED_KEY{"Drive To Speed"};
    inline static const std::string DRIVE_TO_ANGLE_KEY{"Drive To Angle"};

    static constexpr double LOCK_ON_THRESHOLD{2.0};



    void loadFromConfig();
    void reset();
    void startLogging();
    void endLogging();
    void setToBrake();
    void setToCoast();

    void setDriveMode(DriveMode mode);
    void setSysIdMode(SysIdMode mode);

    void setDrive(double translate_x, double translate_y, double turn);
    void setMaxDriveSpeed(double speed);//In meters per second
    double getMaxDriveSpeed();
    void setSpeakerPose(std::string color);

    void stop();
    void autoAim(double controller_x, double controller_y);
    void ringLock(double forward);
    void driveTo(frc::Pose2d pose);
    void updateAnglesSMD();
    bool isAtTarget();
    bool ringLockedOn();
    bool speakerLockedOn();
    double getDistanceFromTarget();
    double getAngleFromTarget();
    double getTime();
    double getAngleToPoint(double x, double y);
    std::optional<double> getDistanceToSpeaker();

    double getRobotAngle();
    double getRobotX();
    units::degree_t getRobotAngleInDegrees();

    void addToLuaState(Keeko::ThreadedScript& script);

    //Lua specific functions
    std::string getCurrentDriveModeAsString();
    void driveToLua(double x, double y, double angle);
    void turnToLua(double angle);
    void setMinDrivePercentage(double percentage);
    void setOdometry(double x, double y, double angle);
    void pointTo(double x, double y);
    double getRobotAngleLua();
    void setFieldOriented(bool is_field);

    //SysyId
    frc2::CommandPtr getDriveSysIdCommand(SysIdMode mode);
    frc2::CommandPtr getRotateSysIdCommand(SysIdMode mode);

    static constexpr double MAX_LIMELIGHT_DISTANCE{1.0};
    static constexpr double LIMELIGHT_MIN_DIV{0.95};

private:
    
    void Periodic() override;
    void doDriveTo();
    void doStop();
    void doTurnTo();
    void doSysId();
    void doAutoAim();
    void doRingLock();

    frc::Pose2d updateOdometry();
    std::unique_ptr<frc2::sysid::SysIdRoutine> sysid_drive_routine;
    std::unique_ptr<frc2::sysid::SysIdRoutine> sysid_rotate_routine;

    std::shared_ptr<RobotContext> context;
    std::unique_ptr<SwerveModuleCordinator> swerve_drive_coordinator;
    std::shared_ptr<MK4SwerveModule> front_left_module;
    std::shared_ptr<MK4SwerveModule> front_right_module;
    std::shared_ptr<MK4SwerveModule> back_left_module;
    std::shared_ptr<MK4SwerveModule> back_right_module;
    std::unique_ptr<CSVLogger<5>> logger;
    std::unique_ptr<frc::SwerveDriveKinematics<4>> kinematics;
    std::unique_ptr<frc::SwerveDrivePoseEstimator<4>> odometry;
    std::chrono::steady_clock::time_point log_start_time;
    PID angle_correction_pid{0.025, 0.0, 0.05};
    double target_angle{0.0};
    double drive_to_proportional{2.0};//TODO: Add setter function
    DriveMode drive_mode{DriveMode::MANUAL};
    SysIdMode sys_id_mode{SysIdMode::IDLE};
    frc::PIDController turn_to_pid{0.0105, 0.0, 0.0};
    frc::PIDController auto_aim_pid{0.0075, 0.0, 0.0};
    double turn_to_tolerance{1.5};
    std::unique_ptr<frc2::CommandPtr> q_f_command;
    std::unique_ptr<frc2::CommandPtr> q_r_command;
    std::unique_ptr<frc2::CommandPtr> d_f_command;
    std::unique_ptr<frc2::CommandPtr> d_r_command;

    double translate_magnitude_deadzone{0.15};//TODO: Make configurable
    double turn_magnitude_deadzone{0.15};//TODO: Make configurable
    bool at_target{false};
    double maxDriveSpeed{4.0};//For smart dashboard
    double minDrivePercentage{0.0};
    double ring_lock_forward{0.0};
    double user_x{0.0};
    double user_y{0.0};

    std::array<double, 2> speaker_pose{0.0, 1.4478};


    frc::Pose2d current_pose;
    frc::Pose2d target_pose;
    std::mutex mutex;
    units::volt_t test_voltage{0.0};
};