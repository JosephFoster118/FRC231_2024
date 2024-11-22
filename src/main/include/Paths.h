#pragma once

#include <filesystem>
#include <frc/Filesystem.h>
#include <wpi/fs.h>

namespace Paths
{
    inline const std::filesystem::path ROBOT_CONFIG = std::filesystem::path(frc::filesystem::GetDeployDirectory())/"config"/"CompConfig.json";
    inline const std::filesystem::path TEST_AUTON = std::filesystem::path(frc::filesystem::GetDeployDirectory())/"scripts"/"auton"/"test_auton.lua";
    inline const std::filesystem::path THREE_RING_LEFT_AUTON = std::filesystem::path(frc::filesystem::GetDeployDirectory())/"scripts"/"auton"/"3_ring_left.lua";
    inline const std::filesystem::path FIVE_RING_AUTON = std::filesystem::path(frc::filesystem::GetDeployDirectory())/"scripts"/"auton"/"5_ring.lua";
    inline const std::filesystem::path FOUR_RING_NEAR_AUTON_2 = std::filesystem::path(frc::filesystem::GetDeployDirectory())/"scripts"/"auton"/"four_ring_near_2.lua";
    inline const std::filesystem::path CENTER_FIELD_AUTON = std::filesystem::path(frc::filesystem::GetDeployDirectory())/"scripts"/"auton"/"center_field.lua";
    inline const std::filesystem::path ONE_SHOT_AUTON = std::filesystem::path(frc::filesystem::GetDeployDirectory())/"scripts"/"auton"/"one_shot.lua";
    inline const std::filesystem::path SHARED_LUA = std::filesystem::path(frc::filesystem::GetDeployDirectory())/"scripts"/"auton"/"shared.lua";
    inline const std::filesystem::path CENTER_CLEAR_LUA = std::filesystem::path(frc::filesystem::GetDeployDirectory())/"scripts"/"auton"/"center_clear.lua";
    inline const std::filesystem::path TEST_LOG = std::filesystem::path(frc::filesystem::GetOperatingDirectory())/"test_log.Pepper";
}
