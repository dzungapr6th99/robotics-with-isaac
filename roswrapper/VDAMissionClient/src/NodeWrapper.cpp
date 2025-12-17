#include "VDA5050Wrapper/NodeWrapper.hpp"
#include "vda5050_msgs/msg/node_position.hpp"
#include "vda5050_msgs/msg/action.hpp"
#include <vector>
#include <string>
#include <rclcpp/rclcpp.hpp>
using NodePosition = vda5050_msgs::msg::NodePosition;
using Action = vda5050_msgs::msg::Action;
NodeWrapper::NodeWrapper()
{
}

NodeWrapper::~NodeWrapper()
{
}

extern "C"
{
    RCLCPP_EXPORT void Node_SetNodeId(NodeWrapper *nodeWrapper, const char *data)
    {
        nodeWrapper->entity.node_id  = data ? data : "";
    }

    RCLCPP_EXPORT void Node_SetSequenceId(NodeWrapper *nodeWrapper, uint32_t sequenceId)
    {
        nodeWrapper->entity.sequence_id = sequenceId;
    }

    RCLCPP_EXPORT void Node_SetNodeDescription(NodeWrapper *nodeWrapper, const char *data)
    {
        nodeWrapper->entity.node_description  = data ? data : "";
    }

    RCLCPP_EXPORT void Node_SetReleased(NodeWrapper *nodeWrapper, bool released)
    {
        nodeWrapper->entity.released = released;
    }

    RCLCPP_EXPORT void Node_SetNodePosition(NodeWrapper* nodeWrapper, const NodePosition* nodePosition)
    {
        nodeWrapper->entity.node_position = *nodePosition;
    }

    RCLCPP_EXPORT void Node_SetAction(NodeWrapper* nodeWrapper, const Action* actions, int length)
    {
        nodeWrapper->entity.actions = std::vector<Action>(actions, actions + length);
    }

}
