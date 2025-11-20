#include "VDA5050Wrapper/NodeWrapper.hpp"
#include "vda5050_msgs/msg/node_position.hpp"
#include "vda5050_msgs/msg/action.hpp"
#include <vector>
#include <string>
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
    void Node_SetNodeId(NodeWrapper *nodeWrapper, const char *data, int length)
    {
        nodeWrapper->entity.node_id.assign(data, length);
    }

    void Node_SetSequenceId(NodeWrapper *nodeWrapper, uint32_t sequenceId)
    {
        nodeWrapper->entity.sequence_id = sequenceId;
    }

    void Node_SetNodeDescription(NodeWrapper *nodeWrapper, const char *data, int length)
    {
        nodeWrapper->entity.node_description.assign(data, length);
    }

    void Node_SetReleased(NodeWrapper *nodeWrapper, bool released)
    {
        nodeWrapper->entity.released = released;
    }

    void Node_SetNodePosition(NodeWrapper* nodeWrapper, const NodePosition* nodePosition)
    {
        nodeWrapper->entity.node_position = *nodePosition;
    }

    void Node_SetAction(NodeWrapper* nodeWrapper, const Action* actions, int length)
    {
        nodeWrapper->entity.actions = std::vector<Action>(actions, actions + length);
    }

}