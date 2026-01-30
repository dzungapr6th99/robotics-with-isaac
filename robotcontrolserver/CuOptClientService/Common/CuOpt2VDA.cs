using System;
using System.Collections.Generic;
using System.Linq;
using ApiObject.Cuopt;
using DbObject;
using VDA5050Message;
using VDA5050Message.Base;

namespace CuOptClientService.Common;

public static class CuOpt2VDA
{
    /// <summary>
    /// Convert CuOpt VRP solver response to VDA5050 orders grouped by vehicle id.
    /// </summary>
    public static Dictionary<string, List<Order>> Convert(CuoptVRPResponse response, IList<Point> points)
    {
        var result = new Dictionary<string, List<Order>>();
        
        var vehicleData = response.Response?.SolverResponse?.VehicleData?.VehicleData ?? response?.Response?.SolverInfeasibleResponse?.VehicleData ?? null;
        if (vehicleData == null || vehicleData.Count == 0)
        {
            return result;
        }

        foreach (var (vehicleId, routeData) in vehicleData)
        {
            var order = new Order
            {
                OrderId = vehicleId,
                OrderUpdateId = 0,
                HeaderId = 0,
                Timestamp = DateTime.UtcNow,
                Version = "2.0",
                Manufacturer = "CuOpt",
                SerialNumber = vehicleId
            };

            var route = routeData.Route ?? new List<int>();
            var taskIds = routeData.TaskIds ?? new List<string>();
            var taskTypes = routeData.Type ?? new List<string>();

            var nodeCount = Math.Max(route.Count, Math.Max(taskIds.Count, taskTypes.Count));
            var nodes = new List<Node>(nodeCount);

            var firstActionIndex = -1;
            for (var i = 0; i < nodeCount; i++)
            {
                var nodeId = route.Count > i ? route[i].ToString() : $"node-{i}";
                var taskId = taskIds.Count > i ? taskIds[i] : null;
                var taskType = taskTypes.Count > i ? taskTypes[i] : null;

                var actionType = ResolveActionType(taskId, taskType);
                var hasAction = !string.IsNullOrWhiteSpace(actionType);

                var node = new Node
                {
                    NodeId = nodeId,
                    // Node sequences are even to interleave with edges for continuity.
                    SequenceId = i * 2,
                    NodeDescription = taskId,
                };

                // attach coordinates if available (matrix index == point index fallback to point id match)
                Point? point = null;
                if (route.Count > i)
                {
                    var idx = route[i];
                    if (idx >= 0 && idx < points.Count)
                        point = points[idx];
                    else
                        point = points.FirstOrDefault(p => p.Id == idx);
                }
                if (point != null)
                {
                    node.NodePosition = new NodePosition
                    {
                        X = point.X ?? 0,
                        Y = point.Y ?? 0,
                        MapId = point.MapId?.ToString() ?? "map"
                    };
                }

                if (hasAction)
                {
                    node.Actions.Add(new VDA5050Message.Base.Action
                    {
                        ActionId = taskId ?? $"action-{i}",
                        ActionType = actionType!,
                        BlockingType = BlockingType.HARD,
                        ActionDescription = taskId
                    });

                    if (firstActionIndex == -1)
                    {
                        firstActionIndex = i;
                    }
                }

                // Released stays true until the first node with an action is encountered.
                node.Released = firstActionIndex == -1 || i < firstActionIndex;

                nodes.Add(node);
            }

            var edges = new List<Edge>(Math.Max(0, nodes.Count - 1));
            for (var i = 1; i < nodes.Count; i++)
            {
                var start = nodes[i - 1];
                var end = nodes[i];
                var edgeReleased = firstActionIndex == -1 || ((i - 1) < firstActionIndex && i < firstActionIndex);

                edges.Add(new Edge
                {
                    EdgeId = $"{start.NodeId}-{end.NodeId}-{i}",
                    // Edges are interleaved between nodes for continuous sequencing.
                    SequenceId = (i * 2) - 1,
                    StartNodeId = start.NodeId,
                    EndNodeId = end.NodeId,
                    Released = edgeReleased
                });
            }

            order.Nodes = nodes;
            order.Edges = edges;

            if (!result.TryGetValue(vehicleId, out var orderList))
            {
                orderList = new List<Order>();
                result[vehicleId] = orderList;
            }

            orderList.Add(order);
        }

        return result;
    }

    private static string? ResolveActionType(string? taskId, string? taskType)
    {
        if (!string.IsNullOrWhiteSpace(taskId))
        {
            if (taskId.Contains("task-charge-", StringComparison.OrdinalIgnoreCase))
            {
                return "Charge";
            }

            if (taskId.Contains("task-delivery-", StringComparison.OrdinalIgnoreCase))
            {
                return "Delivery";
            }
        }

        if (string.Equals(taskType, "Depot", StringComparison.OrdinalIgnoreCase))
        {
            // Treat depot type as charge when no explicit taskId hint is present.
            return "Charge";
        }

        return string.IsNullOrWhiteSpace(taskType) ? null : taskType;
    }
}
