#include "VDA5050Wrapper/EdgeWrapper.hpp"

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
}