#include "VDA5050Wrapper/OrderWrapper.hpp"

using Node = vda5050_msgs::msg::Node;
using Edge = vda5050_msgs::msg::Edge;
OrderWrapper::OrderWrapper()
{
}

OrderWrapper::~OrderWrapper()
{
}

extern "C"
{
    void Order_SetHeaderId(OrderWrapper *orderWrapper, int32_t headerId)
    {
        orderWrapper->entity.header_id = headerId;
    }

    void Order_SetTimeStamp(OrderWrapper *OrderWrapper, const char *data, int length)
    {
        if (!data || length < 0)
        {
            return;
        }
        OrderWrapper->entity.timestamp.assign(data, length);
    }

    void Order_SetVersion(OrderWrapper *OrderWrapper, const char *data, int length)
    {
        if (!data || length < 0)
        {
            return;
        }
        OrderWrapper->entity.version.assign(data, length);
    }

    void Order_SetManufacture(OrderWrapper *OrderWrapper, const char *data, int length)
    {
        if (!data || length < 0)
        {
            return;
        }
        OrderWrapper->entity.manufacturer.assign(data, length);
    }

    void Order_SetSerialNumber(OrderWrapper *OrderWrapper, const char *data, int length)
    {
        if (!data || length < 0)
        {
            return;
        }
        OrderWrapper->entity.serial_number.assign(data, length);
    }

    void Order_SetOrderId(OrderWrapper *OrderWrapper, const char *data, int length)
    {
        if (!data || length < 0)
        {
            return;
        }
        OrderWrapper->entity.order_id.assign(data, length);
    }
    void Order_SetOrderUpdateId(OrderWrapper *orderWrapper, int32_t headerId)
    {
        orderWrapper->entity.order_update_id = headerId;
    }

    void Order_SetZoneSetId(OrderWrapper *OrderWrapper, const char *data, int length)
    {
        if (!data || length < 0)
        {
            return;
        }
        OrderWrapper->entity.zone_set_id.assign(data, length);
    }

    void Order_SetNodes(OrderWrapper *OrderWrapper, const Node *data, int length)
    {
        if (!data || length < 0)
        {
            return;
        }
        OrderWrapper->entity.nodes = std::vector<Node>(data, data + length);
    }

    void Order_SetEgdes(OrderWrapper *OrderWrapper, const Edge *data, int length)
    {
          if (!data || length < 0)
        {
            return;
        }
        OrderWrapper->entity.edges = std::vector<Edge>(data, data + length);
    }
}