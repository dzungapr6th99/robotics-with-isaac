#include "VDA5050Wrapper/OrderWrapper.hpp"
#include <rclcpp/rclcpp.hpp>
#include <vector>
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
    RCLCPP_EXPORT void Order_SetHeaderId(OrderWrapper *orderWrapper, int32_t headerId)
    {
        orderWrapper->entity.header_id = headerId;
    }

    RCLCPP_EXPORT void Order_SetTimeStamp(OrderWrapper *OrderWrapper, const char *data)
    {        
        OrderWrapper->entity.timestamp = data ? data : "";
    }

    RCLCPP_EXPORT void Order_SetVersion(OrderWrapper *OrderWrapper, const char *data)
    {
        OrderWrapper->entity.version = data ? data : "";
    }

    RCLCPP_EXPORT void Order_SetManufacture(OrderWrapper *OrderWrapper, const char *data)
    {
        OrderWrapper->entity.manufacturer = data ? data : "";
    }

    RCLCPP_EXPORT void Order_SetSerialNumber(OrderWrapper *OrderWrapper, const char *data)
    {
        OrderWrapper->entity.serial_number = data ? data : "";
    }

    RCLCPP_EXPORT void Order_SetOrderId(OrderWrapper *OrderWrapper, const char *data)
    {       
        OrderWrapper->entity.order_id = data ? data : "";
    }
    RCLCPP_EXPORT void Order_SetOrderUpdateId(OrderWrapper *orderWrapper, uint32_t headerId)
    {
        orderWrapper->entity.order_update_id = headerId;
    }

    RCLCPP_EXPORT void Order_SetZoneSetId(OrderWrapper *OrderWrapper, const char *data)
    {
        OrderWrapper->entity.zone_set_id = data ? data : "";
    }

    RCLCPP_EXPORT void Order_SetNodes(OrderWrapper *OrderWrapper, const Node *data, int length)
    {
        OrderWrapper->entity.nodes = std::vector<Node>(data, data + length);
    }

    RCLCPP_EXPORT void Order_SetEgdes(OrderWrapper *OrderWrapper, const Edge *data, int length)
    {
        OrderWrapper->entity.edges = std::vector<Edge>(data, data + length);
    }
}
