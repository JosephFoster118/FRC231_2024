#pragma once

#include <vector>
#include <optional>
#include <array>
#include <frc/DriverStation.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <nlohmann/json.hpp>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <Keeko/LuaException.h>
#include <Keeko/Scriptable.h>
#include <Keeko/ThreadedScript.h>

#include "Utility.h"

//Third Party
#include "lua.h"
#include "lauxlib.h"
#include "lualib.h"
#include "LuaBridge.h"

//#include "LimelightHelpers.h"

class Vision
{
public:
    Vision();
    struct SpeakerTargetInfo
    {
        double offset;
        double distance;
    };
    void applyVisionData(frc::SwerveDrivePoseEstimator<4>& estimator);
    std::optional<std::vector<double>> filterData(std::vector<double> raw);
    std::optional<SpeakerTargetInfo> getSpeakerTargetOffset();
    std::optional<double> getRingTargetOffset();

    bool ringInView();
    bool targetInView();
    double targetDistanceFromCenter();
    void setToDriverMode();
    void takeSnapshot();
    bool speakerInView();
    void setToAutoMode();
    void LEDOn();
    void LEDOff();

    void addToLuaState(Keeko::ThreadedScript& script);
    
    static constexpr double Y_OFFSET = 4.105148;
    inline static const std::string LIMELIGHT_TABLE_NAME{"limelight"};
    inline static const std::string INTAKE_LIMELIGHT_TABLE_NAME{"limelight-intake"};
    inline static const std::string RED_BOT_POSE_NAME{"botpose_wpired"};
    inline static const std::string BLUE_BOT_POSE_NAME{"botpose_wpiblue"};
    inline static const std::string PIPELINE_LATENCY_NAME{"tl"};
    inline static const std::string CAPTURE_LATENCY_NAME{"cl"};
    inline static const std::string JSON_ELEMENT_NAME{"json"};
    inline static const std::string JSON_ROOT_NAME{"Results"};
    inline static const std::string JSON_APRIL_SECTION_NAME{"Fiducial"};
    inline static const std::string TARGET_AREA_NAME{"ta"};
    inline static const std::string JSON_APRIL_TAG_ID_NAME{"fID"};
    inline static const std::string JSON_APRIL_TAG_TX_NAME{"tx"};
    inline static const std::string RING_OFFSET_NAME{"tx"};
    inline static const std::string RING_IN_VIEW_NAME{"tv"};
    inline static const std::string TARGET_SPACE_ROBOT_POSE_NAME{"t6r_ts"};


    inline static constexpr std::array<int, 2> SPEAKER_CENTER_TAGS{{4,7}};

private:
    std::array<double, 3> last_reading{{0.0, 0.0, 0.0}};
    frc::Pose2d last_position;

};
