#include "VDA5050Wrapper/EdgeWrapper.hpp"

using Action = vda5050_msgs::msg::Action;
EdgeWrapper::EdgeWrapper()
{
}

EdgeWrapper::~EdgeWrapper()
{
}

extern "C"
{
    void Edge_SetEdgeId(EdgeWrapper *edgeWrapper, const char *data, int length)
    {
        edgeWrapper->entity.edge_id.assign(data, length);
    }

    void Edge_SetSequenceId(EdgeWrapper *edgeWrapper, uint32_t sequenceId)
    {
        edgeWrapper->entity.sequence_id = sequenceId;
    }

    void Edge_SetDescription(EdgeWrapper *edgeWrapper, const char *data, int length)
    {
        edgeWrapper->entity.edge_description.assign(data, length);
    }

    void Edge_SetRelease(EdgeWrapper *edgeWrapper, bool data)
    {
        edgeWrapper->entity.released = data;
    }

    void Edge_SetStartNodeId(EdgeWrapper *edgeWrapper, char *data, int length)
    {
        edgeWrapper->entity.start_node_id.assign(data, length);
    }

    void Edge_SetEndNodeId(EdgeWrapper *edgeWrapper, char *data, int length)
    {
        edgeWrapper->entity.end_node_id.assign(data, length);
    }

    void Edge_SetMaxSpeed(EdgeWrapper *edgeWrapper, double data)
    {
        edgeWrapper->entity.max_speed = data;
    }

    void Edge_SetMaxHeight(EdgeWrapper *edgeWrapper, double data)
    {
        edgeWrapper->entity.max_height = data;
    }

    void Edge_SetMinHeight(EdgeWrapper *edgeWrapper, double minHeight)
    {
        edgeWrapper->entity.min_height = minHeight;
    }

    void Edge_SetOrentation(EdgeWrapper *edgeWrapper, double data)
    {
        edgeWrapper->entity.orientation = data;
    }

    void Edge_SetOrientiationType(EdgeWrapper *edgeWrapper, char *data, int length)
    {
        edgeWrapper->entity.orientation_type.assign(data, length);
    }

    void Edge_SetDirection(EdgeWrapper *edgeWrapper, char *data, int length)
    {
        edgeWrapper->entity.direction.assign(data, length);
    }

    void Edge_SetRotationAllowed(EdgeWrapper *edgeWrapper, bool data)
    {
        edgeWrapper->entity.rotation_allowed = data;
    }

    void Edge_SetMaxRotationSpeed(EdgeWrapper *edgeWrapper, double data)
    {
        edgeWrapper->entity.max_rotation_speed = data;
    }

    void Edge_SetLength(EdgeWrapper *edgeWrapper, double data)
    {
        edgeWrapper->entity.length = data;
    }

     void Edge_SetAction(EdgeWrapper* edgeWrapper, const Action* actions, int length)
    {
        edgeWrapper->entity.actions = std::vector<Action>(actions, actions + length);
    }
}