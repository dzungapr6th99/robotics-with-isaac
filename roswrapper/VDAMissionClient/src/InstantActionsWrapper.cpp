#include "VDA5050Wrapper/InstantActionsWrapper.hpp"
#include <rclcpp/rclcpp.hpp>
#include <vda5050_msgs/msg/action.hpp>

using Action = vda5050_msgs::msg::Action;

InstantActionsWrapper::InstantActionsWrapper()
{
}

InstantActionsWrapper::~InstantActionsWrapper()
{
}

extern "C"
{
    RCLCPP_EXPORT InstantActionsWrapper *InstantActions_Create()
    {
        return new InstantActionsWrapper();
    }

    RCLCPP_EXPORT void InstantActions_Destroy(InstantActionsWrapper *instantActionsWrapper)
    {
        delete instantActionsWrapper;
    }

    RCLCPP_EXPORT void InstantActions_SetHeaderId(InstantActionsWrapper *instantActionsWrapper, int32_t headerId)
    {
        instantActionsWrapper->entity.header_id = headerId;
    }

    RCLCPP_EXPORT void InstantActions_SetTimeStamp(InstantActionsWrapper *instantActionsWrapper, const char *data)
    {
        instantActionsWrapper->entity.timestamp = data ? data : "";
    }

    RCLCPP_EXPORT void InstantActions_SetVersion(InstantActionsWrapper *instantActionsWrapper, const char *data)
    {
        instantActionsWrapper->entity.version = data ? data : "";
    }

    RCLCPP_EXPORT void InstantActions_SetManufacture(InstantActionsWrapper *instantActionsWrapper, const char *data)
    {
        instantActionsWrapper->entity.manufacturer = data ? data : "";
    }

    RCLCPP_EXPORT void InstantActions_SetSerialNumber(InstantActionsWrapper *instantActionsWrapper, const char *data)
    {
        instantActionsWrapper->entity.serial_number = data ? data : "";
    }

    RCLCPP_EXPORT void InstantActions_SetActions(InstantActionsWrapper *instantActionsWrapper, const Action *data, int length)
    {
        instantActionsWrapper->entity.instant_actions = std::vector<Action>(data, data + length);
    }

    RCLCPP_EXPORT void InstantActions_ClearActions(InstantActionsWrapper *instantActionsWrapper)
    {
        instantActionsWrapper->entity.instant_actions.clear();
    }

    RCLCPP_EXPORT void InstantActions_AddAction(InstantActionsWrapper *instantActionsWrapper, const Action *action)
    {
        if (!action)
        {
            return;
        }
        instantActionsWrapper->entity.instant_actions.push_back(*action);
    }
}
