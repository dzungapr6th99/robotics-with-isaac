#include "VDA5050Wrapper/AGVStateWrapper.hpp"
#include <rclcpp/rclcpp.hpp>
#include <vda5050_msgs/msg/node_state.hpp>
#include <vda5050_msgs/msg/edge_state.hpp>
#include <vda5050_msgs/msg/agv_position.hpp>
#include <vda5050_msgs/msg/velocity.hpp>
#include <vda5050_msgs/msg/load.hpp>
#include <vda5050_msgs/msg/action_state.hpp>
#include <vda5050_msgs/msg/battery_state.hpp>
#include <vda5050_msgs/msg/error.hpp>
#include <vda5050_msgs/msg/info.hpp>
#include <vda5050_msgs/msg/safety_state.hpp>

using NodeState = vda5050_msgs::msg::NodeState;
using EdgeState = vda5050_msgs::msg::EdgeState;
using AGVPosition = vda5050_msgs::msg::AGVPosition;
using Velocity = vda5050_msgs::msg::Velocity;
using Load = vda5050_msgs::msg::Load;
using ActionState = vda5050_msgs::msg::ActionState;
using BatteryState = vda5050_msgs::msg::BatteryState;
using Error = vda5050_msgs::msg::Error;
using Info = vda5050_msgs::msg::Info;
using SafetyState = vda5050_msgs::msg::SafetyState;

AGVStateWrapper::AGVStateWrapper()
{
}

AGVStateWrapper::~AGVStateWrapper()
{
}

extern "C"
{
    RCLCPP_EXPORT AGVStateWrapper *AGVState_Create()
    {
        return new AGVStateWrapper();
    }

    RCLCPP_EXPORT void AGVState_Destroy(AGVStateWrapper *wrapper)
    {
        delete wrapper;
    }

    RCLCPP_EXPORT int32_t AGVState_GetHeaderId(AGVStateWrapper *wrapper)
    {
        return wrapper->entity.header_id;
    }

    RCLCPP_EXPORT const char *AGVState_GetTimestamp(AGVStateWrapper *wrapper)
    {
        return wrapper->entity.timestamp.c_str();
    }

    RCLCPP_EXPORT const char *AGVState_GetVersion(AGVStateWrapper *wrapper)
    {
        return wrapper->entity.version.c_str();
    }

    RCLCPP_EXPORT const char *AGVState_GetManufacturer(AGVStateWrapper *wrapper)
    {
        return wrapper->entity.manufacturer.c_str();
    }

    RCLCPP_EXPORT const char *AGVState_GetSerialNumber(AGVStateWrapper *wrapper)
    {
        return wrapper->entity.serial_number.c_str();
    }

    RCLCPP_EXPORT const char *AGVState_GetOrderId(AGVStateWrapper *wrapper)
    {
        return wrapper->entity.order_id.c_str();
    }

    RCLCPP_EXPORT uint32_t AGVState_GetOrderUpdateId(AGVStateWrapper *wrapper)
    {
        return wrapper->entity.order_update_id;
    }

    RCLCPP_EXPORT const char *AGVState_GetZoneSetId(AGVStateWrapper *wrapper)
    {
        return wrapper->entity.zone_set_id.c_str();
    }

    RCLCPP_EXPORT const char *AGVState_GetLastNodeId(AGVStateWrapper *wrapper)
    {
        return wrapper->entity.last_node_id.c_str();
    }

    RCLCPP_EXPORT int32_t AGVState_GetLastNodeSequenceId(AGVStateWrapper *wrapper)
    {
        return wrapper->entity.last_node_sequence_id;
    }

    RCLCPP_EXPORT const NodeState *AGVState_GetNodeStates(AGVStateWrapper *wrapper, int *length)
    {
        if (length)
        {
            *length = static_cast<int>(wrapper->entity.node_states.size());
        }
        return wrapper->entity.node_states.data();
    }

    RCLCPP_EXPORT const NodeState *AGVState_GetNodeStateAt(AGVStateWrapper *wrapper, int index)
    {
        if (!wrapper || index < 0 || index >= static_cast<int>(wrapper->entity.node_states.size()))
        {
            return nullptr;
        }
        return &wrapper->entity.node_states[static_cast<size_t>(index)];
    }

    RCLCPP_EXPORT const EdgeState *AGVState_GetEdgeStates(AGVStateWrapper *wrapper, int *length)
    {
        if (length)
        {
            *length = static_cast<int>(wrapper->entity.edge_states.size());
        }
        return wrapper->entity.edge_states.data();
    }

    RCLCPP_EXPORT const EdgeState *AGVState_GetEdgeStateAt(AGVStateWrapper *wrapper, int index)
    {
        if (!wrapper || index < 0 || index >= static_cast<int>(wrapper->entity.edge_states.size()))
        {
            return nullptr;
        }
        return &wrapper->entity.edge_states[static_cast<size_t>(index)];
    }

    RCLCPP_EXPORT const AGVPosition *AGVState_GetAGVPosition(AGVStateWrapper *wrapper)
    {
        return &wrapper->entity.agv_position;
    }

    RCLCPP_EXPORT const Velocity *AGVState_GetVelocity(AGVStateWrapper *wrapper)
    {
        return &wrapper->entity.velocity;
    }

    RCLCPP_EXPORT const Load *AGVState_GetLoads(AGVStateWrapper *wrapper, int *length)
    {
        if (length)
        {
            *length = static_cast<int>(wrapper->entity.loads.size());
        }
        return wrapper->entity.loads.data();
    }

    RCLCPP_EXPORT const Load *AGVState_GetLoadAt(AGVStateWrapper *wrapper, int index)
    {
        if (!wrapper || index < 0 || index >= static_cast<int>(wrapper->entity.loads.size()))
        {
            return nullptr;
        }
        return &wrapper->entity.loads[static_cast<size_t>(index)];
    }

    RCLCPP_EXPORT bool AGVState_GetDriving(AGVStateWrapper *wrapper)
    {
        return wrapper->entity.driving;
    }

    RCLCPP_EXPORT bool AGVState_GetPaused(AGVStateWrapper *wrapper)
    {
        return wrapper->entity.paused;
    }

    RCLCPP_EXPORT bool AGVState_GetNewBaseRequested(AGVStateWrapper *wrapper)
    {
        return wrapper->entity.new_base_requested;
    }

    RCLCPP_EXPORT double AGVState_GetDistanceSinceLastNode(AGVStateWrapper *wrapper)
    {
        return wrapper->entity.distance_since_last_node;
    }

    RCLCPP_EXPORT const ActionState *AGVState_GetActionStates(AGVStateWrapper *wrapper, int *length)
    {
        if (length)
        {
            *length = static_cast<int>(wrapper->entity.action_states.size());
        }
        return wrapper->entity.action_states.data();
    }

    RCLCPP_EXPORT const ActionState *AGVState_GetActionStateAt(AGVStateWrapper *wrapper, int index)
    {
        if (!wrapper || index < 0 || index >= static_cast<int>(wrapper->entity.action_states.size()))
        {
            return nullptr;
        }
        return &wrapper->entity.action_states[static_cast<size_t>(index)];
    }

    RCLCPP_EXPORT const BatteryState *AGVState_GetBatteryState(AGVStateWrapper *wrapper)
    {
        return &wrapper->entity.battery_state;
    }

    RCLCPP_EXPORT const char *AGVState_GetOperatingMode(AGVStateWrapper *wrapper)
    {
        return wrapper->entity.operating_mode.c_str();
    }

    RCLCPP_EXPORT const Error *AGVState_GetErrors(AGVStateWrapper *wrapper, int *length)
    {
        if (length)
        {
            *length = static_cast<int>(wrapper->entity.errors.size());
        }
        return wrapper->entity.errors.data();
    }

    RCLCPP_EXPORT const Error *AGVState_GetErrorAt(AGVStateWrapper *wrapper, int index)
    {
        if (!wrapper || index < 0 || index >= static_cast<int>(wrapper->entity.errors.size()))
        {
            return nullptr;
        }
        return &wrapper->entity.errors[static_cast<size_t>(index)];
    }

    RCLCPP_EXPORT const Info *AGVState_GetInformations(AGVStateWrapper *wrapper, int *length)
    {
        if (length)
        {
            *length = static_cast<int>(wrapper->entity.informations.size());
        }
        return wrapper->entity.informations.data();
    }

    RCLCPP_EXPORT const Info *AGVState_GetInformationAt(AGVStateWrapper *wrapper, int index)
    {
        if (!wrapper || index < 0 || index >= static_cast<int>(wrapper->entity.informations.size()))
        {
            return nullptr;
        }
        return &wrapper->entity.informations[static_cast<size_t>(index)];
    }

    RCLCPP_EXPORT const SafetyState *AGVState_GetSafetyState(AGVStateWrapper *wrapper)
    {
        return &wrapper->entity.safety_state;
    }
}
