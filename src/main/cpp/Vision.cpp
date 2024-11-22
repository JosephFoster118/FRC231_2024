#include "Vision.h"
#include <iostream>

Vision::Vision()
{
    frc::SmartDashboard::SetDefaultNumber("xy_std", 0.0);
    frc::SmartDashboard::SetDefaultNumber("deg_std", 0.0);
    frc::SmartDashboard::SetDefaultNumber("cam_offset", 0.0);
    frc::SmartDashboard::SetDefaultNumber("target_distance", 0.0);
}

std::optional<std::vector<double>> Vision::filterData(std::vector<double> raw)
{
    if(raw[0] == 0.0 && raw[1] == 0.0 && raw[2] == 0.0)
    {
        return std::optional<std::vector<double>>{};
    }
    if(raw[0] == last_reading[0] &&
        raw[1] == last_reading[1] &&
        raw[5] == last_reading[2])
    {
        return std::optional<std::vector<double>>{};
    }
    last_reading = {raw[0], raw[1], raw[5]};

    return std::optional<std::vector<double>>{{ raw[0] , raw[1] - Y_OFFSET, raw[5]}};
}


void Vision::applyVisionData(frc::SwerveDrivePoseEstimator<4>& estimator)
{
    auto current_alliance = frc::DriverStation::GetAlliance();
    if(!current_alliance.has_value())
    {
        frc::SmartDashboard::PutNumber("xy_std", -231);
        frc::SmartDashboard::PutNumber("deg_std", -231);
        return;
    }
    try
    {
       // std::cout << nt::NetworkTableInstance::GetDefault().GetTable(LIMELIGHT_TABLE_NAME)->GetString(JSON_ELEMENT_NAME,"")               << std::endl;
        std::string limelight_info = nt::NetworkTableInstance::GetDefault().GetTable(LIMELIGHT_TABLE_NAME)->GetString(JSON_ELEMENT_NAME,"");
        if(limelight_info.empty())
        {
            frc::SmartDashboard::PutNumber("xy_std", -118);
            frc::SmartDashboard::PutNumber("deg_std", -118);
            return;
        }
        nlohmann::json limelight_info_json;
        limelight_info_json = nlohmann::json::parse(limelight_info);
    
        int number_of_targets = limelight_info_json[JSON_ROOT_NAME][JSON_APRIL_SECTION_NAME].size();
        if(number_of_targets < 1)
        {
            frc::SmartDashboard::PutNumber("xy_std", -148);
            frc::SmartDashboard::PutNumber("deg_std", -148);
            return;
        }
        double target_area = 0.1;// limelight_info_json[JSON_ROOT_NAME][JSON_APRIL_SECTION_NAME][0][TARGET_AREA_NAME];
        auto robot_estimate_position_raw = estimator.GetEstimatedPosition();
        double latency = (
            nt::NetworkTableInstance::GetDefault().GetTable(LIMELIGHT_TABLE_NAME)->GetNumber(PIPELINE_LATENCY_NAME, 0) +
            nt::NetworkTableInstance::GetDefault().GetTable(LIMELIGHT_TABLE_NAME)->GetNumber(CAPTURE_LATENCY_NAME, 0)
        )/1000;
        frc::Pose2d vision_position;
        if(current_alliance == frc::DriverStation::kBlue)
        {
            auto raw_position = nt::NetworkTableInstance::GetDefault().GetTable(LIMELIGHT_TABLE_NAME)->GetNumberArray(BLUE_BOT_POSE_NAME, std::vector<double>(6));
            vision_position = frc::Pose2d(units::meter_t{raw_position[0]}, units::meter_t{raw_position[1] - Y_OFFSET}, frc::Rotation2d(units::degree_t{raw_position[5]}));
        }
        else
        {
            auto raw_position = nt::NetworkTableInstance::GetDefault().GetTable(LIMELIGHT_TABLE_NAME)->GetNumberArray(RED_BOT_POSE_NAME, std::vector<double>(6));
            vision_position = frc::Pose2d(units::meter_t{raw_position[0]}, units::meter_t{raw_position[1] - Y_OFFSET}, frc::Rotation2d(units::degree_t{raw_position[5]}));
        }
        if(vision_position.X().value() == 0.0)
        {
            frc::SmartDashboard::PutNumber("xy_std", -231);
            frc::SmartDashboard::PutNumber("deg_std", -231);
            return;
        }
        if(vision_position == last_position)
        {
            frc::SmartDashboard::PutNumber("xy_std", -231);
            frc::SmartDashboard::PutNumber("deg_std", -231);
            return;
        }
        last_position = vision_position;
        double distance_between_cord_and_vision = robot_estimate_position_raw.Translation().Distance(vision_position.Translation()).value();
        //Get std div
        double xy_stds;
        double deg_stds;
        if(number_of_targets >= 2)//Multiple targets detected
        {
            xy_stds = 0.96;
            deg_stds = 6;
        }
        // else if(target_area > 0.6 && distance_between_cord_and_vision < 0.5)//Close to target
        // {
        //     xy_stds = 1.2;
        //     deg_stds = 12;
        // }
        else if(target_area > 0.05 && distance_between_cord_and_vision < 0.5)//Far from target
        {
            xy_stds = 2.0;
            deg_stds = 30;
        }
        else
        {
            frc::SmartDashboard::PutNumber("xy_std", -231);
            frc::SmartDashboard::PutNumber("deg_std", -231);
            return;
        }
        frc::SmartDashboard::PutNumber("xy_std", xy_stds);
        frc::SmartDashboard::PutNumber("deg_std", deg_stds);
        estimator.SetVisionMeasurementStdDevs({xy_stds, xy_stds, Utility::toRadians(deg_stds)});
        estimator.AddVisionMeasurement(vision_position, frc::Timer::GetFPGATimestamp() - units::second_t{latency});
    }
    catch(const std::exception& e)
    {
        //std::cout << "Error getting info from limelight!" << std::endl;
       //+ std::cerr << e.what() << '\n';
        return;
    }
}

std::optional<Vision::SpeakerTargetInfo> Vision::getSpeakerTargetOffset()
{
    try
    {
        std::string limelight_info = nt::NetworkTableInstance::GetDefault().GetTable(LIMELIGHT_TABLE_NAME)->GetString(JSON_ELEMENT_NAME,"");
        nlohmann::json limelight_info_json;
        limelight_info_json = nlohmann::json::parse(limelight_info);
        int number_of_targets = limelight_info_json[JSON_ROOT_NAME][JSON_APRIL_SECTION_NAME].size();
        if(number_of_targets == 0)
        {
            frc::SmartDashboard::PutNumber("cam_offset", -231);
            return std::nullopt;
        }
        std::optional<nlohmann::json> selected_target = std::nullopt;
        for(const auto& tag: limelight_info_json[JSON_ROOT_NAME][JSON_APRIL_SECTION_NAME])
        {
            if(!tag[JSON_APRIL_TAG_ID_NAME].is_number_integer())
            {
                std::cout << "Tag id isnt an integer" << std::endl;
                continue;
            }
            int tag_id = tag[JSON_APRIL_TAG_ID_NAME];
            bool tag_found = false;
            for(const auto& valid_ids: SPEAKER_CENTER_TAGS)
            {
                if(tag_id == valid_ids)
                {
                    tag_found = true;
                    break;
                }
            }
            if(tag_found)
            {
                selected_target = tag;
                break;
            }
        }
        if(!selected_target.has_value())
        {
            frc::SmartDashboard::PutNumber("cam_offset", -148);
            return std::nullopt; //No valid target found
        }
        if(!selected_target.value()[JSON_APRIL_TAG_TX_NAME].is_number())
        {
            //std::cout << "tx isnt a number?" << std::endl;
        }
        //std::cout << "target offset " << selected_target.value()[JSON_APRIL_TAG_TX_NAME] << std::endl;
        frc::SmartDashboard::PutNumber("cam_offset", selected_target.value()[JSON_APRIL_TAG_TX_NAME]);
        auto current_alliance = frc::DriverStation::GetAlliance();
        double distance = 0.0;
        if(current_alliance == frc::DriverStation::kBlue)
        {
            auto raw_position = nt::NetworkTableInstance::GetDefault().GetTable(LIMELIGHT_TABLE_NAME)->GetNumberArray(BLUE_BOT_POSE_NAME, std::vector<double>(6));
            double x = raw_position[0];
            double y = raw_position[1] - Y_OFFSET + 1.45;

            distance = std::sqrt(x*x + y*y);
        }
        else
        {
            auto raw_position = nt::NetworkTableInstance::GetDefault().GetTable(LIMELIGHT_TABLE_NAME)->GetNumberArray(RED_BOT_POSE_NAME, std::vector<double>(6));
            double x = raw_position[0];
            double y = raw_position[1] - Y_OFFSET - 1.45;
            distance = std::sqrt(x*x + y*y);
        }
        
        frc::SmartDashboard::PutNumber("target_distance", distance);
        return std::optional<Vision::SpeakerTargetInfo>{{selected_target.value()[JSON_APRIL_TAG_TX_NAME], distance}};
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        return std::nullopt;
    }
}

bool Vision::ringInView()
{
    return nt::NetworkTableInstance::GetDefault().GetTable(INTAKE_LIMELIGHT_TABLE_NAME)->GetEntry(RING_IN_VIEW_NAME).GetInteger(0) == 1;
}

bool Vision::targetInView()
{
    return false;// TODO: Implement
}

double Vision::targetDistanceFromCenter()
{
    return 0.0;// TODO: Implement
}

void Vision::addToLuaState(Keeko::ThreadedScript& script)
{
    luabridge::getGlobalNamespace(script)
        .beginClass<Vision> ("Vision")
            .addFunction("ringInView", &Vision::ringInView)
            .addFunction("targetInView", &Vision::targetInView)
            .addFunction("targetDistanceFromCenter", &Vision::targetDistanceFromCenter)
            .addFunction("takeSnapshot", &Vision::takeSnapshot)
        .endClass();
    script.addInstance("vision", this);
}

std::optional<double> Vision::getRingTargetOffset()
{
    if(!ringInView())
    {
        frc::SmartDashboard::PutNumber("cam_offset", -118);
        return std::nullopt;
    }
    double offset = nt::NetworkTableInstance::GetDefault().GetTable(INTAKE_LIMELIGHT_TABLE_NAME)->GetEntry(RING_OFFSET_NAME).GetDouble(0);
    frc::SmartDashboard::PutNumber("cam_offset", offset);
    return offset;
}

void Vision::setToDriverMode()
{
    nt::NetworkTableInstance::GetDefault().GetTable(INTAKE_LIMELIGHT_TABLE_NAME)->PutNumber("camMode", 1);
}

void Vision::setToAutoMode()
{
    nt::NetworkTableInstance::GetDefault().GetTable(INTAKE_LIMELIGHT_TABLE_NAME)->PutNumber("camMode", 0);
}

bool Vision::speakerInView()
{
    try
    {
        std::string limelight_info = nt::NetworkTableInstance::GetDefault().GetTable(LIMELIGHT_TABLE_NAME)->GetString(JSON_ELEMENT_NAME,"");
        nlohmann::json limelight_info_json;
        limelight_info_json = nlohmann::json::parse(limelight_info);
        int number_of_targets = limelight_info_json[JSON_ROOT_NAME][JSON_APRIL_SECTION_NAME].size();
        if(number_of_targets == 0)
        {
            return false;
        }
        std::optional<nlohmann::json> selected_target = std::nullopt;
        for(const auto& tag: limelight_info_json[JSON_ROOT_NAME][JSON_APRIL_SECTION_NAME])
        {
            if(!tag[JSON_APRIL_TAG_ID_NAME].is_number_integer())
            {
                std::cout << "Tag id isnt an integer" << std::endl;
                continue;
            }
            int tag_id = tag[JSON_APRIL_TAG_ID_NAME];
            bool tag_found = false;
            for(const auto& valid_ids: SPEAKER_CENTER_TAGS)
            {
                if(tag_id == valid_ids)
                {
                    tag_found = true;
                    break;
                }
            }
            if(tag_found)
            {
                selected_target = tag;
                break;
            }
        }
        if(!selected_target.has_value())
        {
            return false; //No valid target found
        }
        
        return true;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        return false;
    }
}

void Vision::takeSnapshot()
{
    nt::NetworkTableInstance::GetDefault().GetTable(INTAKE_LIMELIGHT_TABLE_NAME)->PutNumber("snapshot", 1);
}

void Vision::LEDOn()
{
    nt::NetworkTableInstance::GetDefault().GetTable(LIMELIGHT_TABLE_NAME)->PutNumber("ledMode", 3);
}

void Vision::LEDOff()
{
    nt::NetworkTableInstance::GetDefault().GetTable(LIMELIGHT_TABLE_NAME)->PutNumber("ledMode", 1);
}
