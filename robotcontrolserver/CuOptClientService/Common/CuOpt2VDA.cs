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
    /// Orders are split by action boundaries; each order releases nodes/edges up to the first action in that order.
    /// </summary>
    public static Dictionary<string, List<Order>> Convert(CuoptVRPResponse response, IList<Point> points)
    {
        var result = new Dictionary<string, List<Order>>();

        var vehicleData = response.Response?.SolverResponse?.VehicleData?.VehicleData
                         ?? response.Response?.SolverInfeasibleResponse?.VehicleData
                         ?? null;
        if (vehicleData == null || vehicleData.Count == 0)
            return result;

        foreach (var (vehicleId, routeData) in vehicleData)
        {
            var route = routeData.Route ?? new List<int>();
            var taskIds = routeData.TaskIds ?? new List<string>();
            var taskTypes = routeData.Type ?? new List<string>();

            var allNodes = BuildNodesWithActions(route, taskIds, taskTypes, points);
            var actionIndices = Enumerable.Range(0, allNodes.Count)
                                          .Where(i => allNodes[i].Actions.Any())
                                          .ToList();

            var ordersForVehicle = new List<Order>();

            if (actionIndices.Count == 0)
            {
                // no action => single order, everything released
                ordersForVehicle.Add(BuildOrderSegment(vehicleId, allNodes, 0, allNodes.Count - 1, null));
            }
            else
            {
                int start = 0;
                for (int ai = 0; ai < actionIndices.Count; ai++)
                {
                    int end = actionIndices[ai];
                    ordersForVehicle.Add(BuildOrderSegment(vehicleId, allNodes, start, end, actionIndices[ai]));
                    start = end; // next segment starts at this action node
                }

                if (start < allNodes.Count - 1)
                {
                    ordersForVehicle.Add(BuildOrderSegment(vehicleId, allNodes, start, allNodes.Count - 1, null));
                }
            }

            if (!result.TryGetValue(vehicleId, out var existing))
            {
                result[vehicleId] = ordersForVehicle;
            }
            else
            {
                existing.AddRange(ordersForVehicle);
            }
        }

        return result;
    }

    private static List<Node> BuildNodesWithActions(
        List<int> route,
        List<string> taskIds,
        List<string> taskTypes,
        IList<Point> points)
    {
        var nodeCount = Math.Max(route.Count, Math.Max(taskIds.Count, taskTypes.Count));
        var nodes = new List<Node>(nodeCount);

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
                SequenceId = i * 2, // temp, will be renumbered per order
                NodeDescription = taskId,
            };

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
            }

            nodes.Add(node);
        }

        return nodes;
    }

    private static Order BuildOrderSegment(string vehicleId, List<Node> allNodes, int startIdx, int endIdx, int? firstActionIdx)
    {
        var order = new Order
        {
            OrderId = $"{vehicleId}-seg-{startIdx}-{endIdx}",
            OrderUpdateId = 0,
            HeaderId = 0,
            Timestamp = DateTime.UtcNow,
            Version = "2.0",
            Manufacturer = "CuOpt",
            SerialNumber = vehicleId
        };

        var segmentNodes = new List<Node>();
        for (int local = 0, i = startIdx; i <= endIdx; i++, local++)
        {
            var clone = DeepCloneNode(allNodes[i]);
            clone.SequenceId = local * 2;
            clone.Released = true; // release all nodes in this order

            segmentNodes.Add(clone);
        }

        var segmentEdges = new List<Edge>();
        for (int localEdge = 1; localEdge < segmentNodes.Count; localEdge++)
        {
            var start = segmentNodes[localEdge - 1];
            var end = segmentNodes[localEdge];

            segmentEdges.Add(new Edge
            {
                EdgeId = $"{start.NodeId}-{end.NodeId}-{localEdge}",
                SequenceId = (localEdge * 2) - 1,
                StartNodeId = start.NodeId,
                EndNodeId = end.NodeId,
                Released = true
            });
        }

        order.Nodes = segmentNodes;
        order.Edges = segmentEdges;
        return order;
    }

    private static Node DeepCloneNode(Node src)
    {
        var node = new Node
        {
            NodeId = src.NodeId,
            SequenceId = src.SequenceId,
            NodeDescription = src.NodeDescription,
            Released = src.Released,
            NodePosition = src.NodePosition == null ? null : new NodePosition
            {
                X = src.NodePosition.X,
                Y = src.NodePosition.Y,
                Theta = src.NodePosition.Theta,
                AllowedDeviationTheta = src.NodePosition.AllowedDeviationTheta,
                AllowedDeviationXY = src.NodePosition.AllowedDeviationXY,
                MapId = src.NodePosition.MapId,
                MapDescription = src.NodePosition.MapDescription
            }
        };

        foreach (var act in src.Actions)
        {
            node.Actions.Add(new VDA5050Message.Base.Action
            {
                ActionId = act.ActionId,
                ActionType = act.ActionType,
                ActionDescription = act.ActionDescription,
                BlockingType = act.BlockingType,
                ActionParameters = act.ActionParameters == null
                    ? null
                    : act.ActionParameters.Select(p => new ActionParameter { Key = p.Key, Value = p.Value }).ToList()
            });
        }

        return node;
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

        // ignore walk placeholder actions
        if (string.Equals(taskType, "w", StringComparison.OrdinalIgnoreCase))
            return null;

        if (string.Equals(taskType, "Depot", StringComparison.OrdinalIgnoreCase))
        {
            // Treat depot type as charge when no explicit taskId hint is present.
            return "Pick";
        }

        return string.IsNullOrWhiteSpace(taskType) ? null : taskType;
    }
}
