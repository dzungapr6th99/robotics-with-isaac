#include "VDA5050Wrapper/NodeStateWrapper.hpp"
#include <rclcpp/rclcpp.hpp>
#include <vda5050_msgs/msg/node_position.hpp>

using NodePosition = vda5050_msgs::msg::NodePosition;

NodeStateWrapper::NodeStateWrapper()
{
}

NodeStateWrapper::~NodeStateWrapper()
{
}

extern "C"
{
    RCLCPP_EXPORT const char *NodeState_GetNodeId(NodeStateWrapper *wrapper)
    {
        return wrapper->entity.node_id.c_str();
    }

    RCLCPP_EXPORT int32_t NodeState_GetSequenceId(NodeStateWrapper *wrapper)
    {
        return wrapper->entity.sequence_id;
    }

    RCLCPP_EXPORT const char *NodeState_GetNodeDescription(NodeStateWrapper *wrapper)
    {
        return wrapper->entity.node_description.c_str();
    }

    RCLCPP_EXPORT const NodePosition *NodeState_GetPosition(NodeStateWrapper *wrapper)
    {
        return &wrapper->entity.position;
    }

    RCLCPP_EXPORT bool NodeState_GetReleased(NodeStateWrapper *wrapper)
    {
        return wrapper->entity.released;
    }
}
