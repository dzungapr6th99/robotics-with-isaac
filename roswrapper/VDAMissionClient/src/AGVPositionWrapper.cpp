#include "VDA5050Wrapper/AGVPositionWrapper.hpp"
#include <rclcpp/rclcpp.hpp>

AGVPositionWrapper::AGVPositionWrapper()
{
}

AGVPositionWrapper::~AGVPositionWrapper()
{
}

extern "C"
{
    RCLCPP_EXPORT AGVPositionWrapper *AGVPosition_Create()
    {
        return new AGVPositionWrapper();
    }

    RCLCPP_EXPORT void AGVPosition_Destroy(AGVPositionWrapper *wrapper)
    {
        delete wrapper;
    }

    RCLCPP_EXPORT bool AGVPosition_GetPositionInitialized(AGVPositionWrapper *wrapper)
    {
        return wrapper->entity.position_initialized;
    }

    RCLCPP_EXPORT double AGVPosition_GetLocalizationScore(AGVPositionWrapper *wrapper)
    {
        return wrapper->entity.localization_score;
    }

    RCLCPP_EXPORT double AGVPosition_GetDeviationRange(AGVPositionWrapper *wrapper)
    {
        return wrapper->entity.deviation_range;
    }

    RCLCPP_EXPORT double AGVPosition_GetX(AGVPositionWrapper *wrapper)
    {
        return wrapper->entity.x;
    }

    RCLCPP_EXPORT double AGVPosition_GetY(AGVPositionWrapper *wrapper)
    {
        return wrapper->entity.y;
    }

    RCLCPP_EXPORT double AGVPosition_GetTheta(AGVPositionWrapper *wrapper)
    {
        return wrapper->entity.theta;
    }

    RCLCPP_EXPORT const char *AGVPosition_GetMapId(AGVPositionWrapper *wrapper)
    {
        return wrapper->entity.map_id.c_str();
    }

    RCLCPP_EXPORT const char *AGVPosition_GetMapDescription(AGVPositionWrapper *wrapper)
    {
        return wrapper->entity.map_description.c_str();
    }
}
