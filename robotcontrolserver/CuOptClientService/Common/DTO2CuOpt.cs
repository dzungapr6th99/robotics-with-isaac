using System;
using System.Collections.Generic;
using System.Linq;
using ApiObject.Cuopt;
using DbObject;
using ShareMemoryData;

namespace CuOptClientService.Common;

public static class DTO2CuOpt
{
    public static List<List<double>> CreateCostmapMatrix(List<Point> points, List<Route> routes)
    {
        var matrixSize = points.Count;
        var result = new double[matrixSize, matrixSize];

        // Map point id to matrix index.
        var indexByPointId = new Dictionary<int, int>();
        for (var i = 0; i < points.Count; i++)
        {
            var id = points[i].Id ?? i;
            indexByPointId[id] = i;
        }

        const int unreachable = 1_000_000; // large cost for unreachable pairs

        // Initialize matrix with unreachable except diagonals.
        for (var i = 0; i < matrixSize; i++)
        {
            for (var j = 0; j < matrixSize; j++)
            {
                result[i, j] = i == j ? 0 : unreachable;
            }
        }

        if (routes == null || routes.Count == 0)
        {
            return ToList(result);
        }

        foreach (var route in routes)
        {
            if (route.FromPointId == null || route.ToPointId == null)
            {
                continue;
            }

            if (!indexByPointId.TryGetValue(route.FromPointId.Value, out var fromIdx) ||
                !indexByPointId.TryGetValue(route.ToPointId.Value, out var toIdx))
            {
                continue;
            }

            var fromPoint = route.FromPoint ?? points.FirstOrDefault(p => p.Id == route.FromPointId);
            var toPoint = route.ToPoint ?? points.FirstOrDefault(p => p.Id == route.ToPointId);

            var cost = CalculateDistance(fromPoint, toPoint);

            // Assume undirected cost matrix (symmetric) unless domain dictates otherwise.
            result[fromIdx, toIdx] = Math.Min(result[fromIdx, toIdx], cost);
            result[toIdx, fromIdx] = Math.Min(result[toIdx, fromIdx], cost);
        }

        return ToList(result);
    }

    /// <summary>
    /// Build cuOpt graph-solve request (CSR waypoint graph) from domain objects.
    /// </summary>
    public static CuOptGraphSolveRequest CreateGraphSolveRequest(
        List<Point> points,
        List<Route> routes,
        List<RobotStatus> robots,
        List<RobotTask> tasks,
        int timeLimitSeconds = 1,
        bool bidirectional = true)
    {

        // Build point index for CSR.
        var pointIndex = new Dictionary<int, int>();
        for (var i = 0; i < points.Count; i++)
        {
            var id = points[i].Id ?? i;
            if (!pointIndex.ContainsKey(id))
            {
                pointIndex[id] = i;
            }
        }

        var nodeCount = points.Count;
        var perNode = new List<(int to, double w)>[nodeCount];
        for (var i = 0; i < nodeCount; i++) perNode[i] = new List<(int to, double w)>();

        foreach (var route in routes)
        {
            if (route.FromPointId == null || route.ToPointId == null) continue;
            if (!pointIndex.TryGetValue(route.FromPointId.Value, out var fromIdx)) continue;
            if (!pointIndex.TryGetValue(route.ToPointId.Value, out var toIdx)) continue;

            var fromPt = route.FromPoint ?? points.FirstOrDefault(p => p.Id == route.FromPointId);
            var toPt = route.ToPoint ?? points.FirstOrDefault(p => p.Id == route.ToPointId);
            var w = CalculateDistance(fromPt, toPt);

            perNode[fromIdx].Add((toIdx, w));
            if (bidirectional)
            {
                perNode[toIdx].Add((fromIdx, w));
            }
        }

        // Build CSR arrays.
        var offsets = new List<int>(nodeCount + 1) { 0 };
        var edges = new List<int>();
        var weights = new List<double>();
        for (var i = 0; i < nodeCount; i++)
        {
            var list = perNode[i];
            foreach (var edge in list)
            {
                edges.Add(edge.to);
                weights.Add(edge.w);
            }
            offsets.Add(edges.Count);
        }

        var waypointGraph = new WaypointGraph
        {
            Offsets = offsets,
            Edges = edges,
            Weights = weights
        };

        // Vehicle data
        var vehicleLocations = new List<List<int>>();
        foreach (var robot in robots)
        {
            var startIdx = FindNearestPointIndex(robot.CoordinateX, robot.CoordinateY, points);
            vehicleLocations.Add(new List<int> { startIdx, startIdx });
        }

        var capacities = robots
            .Select(r => Math.Max(1, r.RemainCapacity > 0 ? r.RemainCapacity : (r.MaxCapacity > 0 ? r.MaxCapacity : 1)))
            .Select(c => new List<int> { c })
            .ToList();

        var vehicleIds = robots
            .Select(r => string.IsNullOrWhiteSpace(r.SerialNumber) ? Guid.NewGuid().ToString() : r.SerialNumber)
            .ToList();

        // Tasks
        var taskLocations = new List<int>();
        var demandPerCapacity = new List<int>();
        var taskIds = new List<string>();

        for (var i = 0; i < tasks.Count; i++)
        {
            var t = tasks[i];
            int chosenPointId = t.DeliveryToPointId ?? t.DeliveryFromPointId ?? pointIndex.Keys.FirstOrDefault();
            var locIdx = pointIndex.TryGetValue(chosenPointId, out var idx) ? idx : 0;
            taskLocations.Add(locIdx);
            var taskId = !string.IsNullOrWhiteSpace(t.Name)
                ? t.Name!
                : (t.Id.HasValue ? $"task-{t.Id.Value}" : $"task-{i}");
            taskIds.Add(taskId);
            demandPerCapacity.Add(Math.Max(1, t.Capacity ?? 1));
        }

        return new CuOptGraphSolveRequest
        {
            CostWaypointGraphData = new CostWaypointGraphData
            {
                WaypointGraph = new Dictionary<string, WaypointGraph>
                {
                    { "0", waypointGraph }
                }
            },
            FleetData = new FleetData
            {
                VehicleIds = vehicleIds,
                VehicleLocations = vehicleLocations,
                Capacities = capacities,
                VehicleTimeWindows = null
            },
            TaskData = new TaskData
            {
                TaskIds = taskIds,
                TaskLocations = taskLocations,
                Demand = new List<List<int>> { demandPerCapacity },
                TaskTimeWindows = null,
                ServiceTimes = null
            },
            SolverConfig = new SolverConfig
            {
                TimeLimit = timeLimitSeconds
            }
        };
    }

    private static int FindNearestPointIndex(double x, double y, List<Point> points)
    {
        if (points.Count == 0) return 0;
        var bestIdx = 0;
        double bestDist = double.MaxValue;
        for (var i = 0; i < points.Count; i++)
        {
            var p = points[i];
            if (p.X == null || p.Y == null) continue;
            var dx = p.X.Value - x;
            var dy = p.Y.Value - y;
            var d = dx * dx + dy * dy;
            if (d < bestDist)
            {
                bestDist = d;
                bestIdx = i;
            }
        }
        return bestIdx;
    }

    private static double CalculateDistance(Point? a, Point? b)
    {
        if (a?.X == null || a?.Y == null || b?.X == null || b?.Y == null)
        {
            return 1;
        }

        var dx = a.X.Value - b.X.Value;
        var dy = a.Y.Value - b.Y.Value;
        // Use Euclidean distance and round up to int cost.
        return Math.Sqrt((dx * dx) + (dy * dy));
    }

    private static List<List<double>> ToList(double[,] matrix)
    {
        var rows = matrix.GetLength(0);
        var cols = matrix.GetLength(1);
        var list = new List<List<double>>(rows);
        for (var i = 0; i < rows; i++)
        {
            var row = new List<double>(cols);
            for (var j = 0; j < cols; j++)
            {
                row.Add(matrix[i, j]);
            }

            list.Add(row);
        }

        return list;
    }

    
}
