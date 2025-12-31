#include "VDA5050Wrapper/BatteryStateWrapper.hpp"
#include <rclcpp/rclcpp.hpp>

BatteryStateWrapper::BatteryStateWrapper()
{
}

BatteryStateWrapper::~BatteryStateWrapper()
{
}

extern "C"
{
    RCLCPP_EXPORT BatteryStateWrapper *BatteryState_Create()
    {
        return new BatteryStateWrapper();
    }

    RCLCPP_EXPORT void BatteryState_Destroy(BatteryStateWrapper *wrapper)
    {
        delete wrapper;
    }

    RCLCPP_EXPORT double BatteryState_GetBatteryCharge(BatteryStateWrapper *wrapper)
    {
        return wrapper->entity.battery_charge;
    }

    RCLCPP_EXPORT double BatteryState_GetBatteryVoltage(BatteryStateWrapper *wrapper)
    {
        return wrapper->entity.battery_voltage;
    }

    RCLCPP_EXPORT int8_t BatteryState_GetBatteryHealth(BatteryStateWrapper *wrapper)
    {
        return wrapper->entity.battery_health;
    }

    RCLCPP_EXPORT bool BatteryState_GetCharging(BatteryStateWrapper *wrapper)
    {
        return wrapper->entity.charging;
    }

    RCLCPP_EXPORT uint32_t BatteryState_GetReach(BatteryStateWrapper *wrapper)
    {
        return wrapper->entity.reach;
    }
}
