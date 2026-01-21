using System;
using DbObject;

namespace CuOptClientService.Common;

public static class DTO2CuOpt
{
    public static List<List<int>> CreateCostmapMatrix(List<Point> poinst, List<Route> routes)
    {
        var points = poinst ?? new List<Point>();
        var matrixSize = points.Count;
        var result = new int[matrixSize, matrixSize];

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

    private static int CalculateDistance(Point? a, Point? b)
    {
        if (a?.X == null || a?.Y == null || b?.X == null || b?.Y == null)
        {
            return 1;
        }

        var dx = a.X.Value - b.X.Value;
        var dy = a.Y.Value - b.Y.Value;
        // Use Euclidean distance and round up to int cost.
        return (int)Math.Ceiling(Math.Sqrt((dx * dx) + (dy * dy)));
    }

    private static List<List<int>> ToList(int[,] matrix)
    {
        var rows = matrix.GetLength(0);
        var cols = matrix.GetLength(1);
        var list = new List<List<int>>(rows);
        for (var i = 0; i < rows; i++)
        {
            var row = new List<int>(cols);
            for (var j = 0; j < cols; j++)
            {
                row.Add(matrix[i, j]);
            }

            list.Add(row);
        }

        return list;
    }

    
}
