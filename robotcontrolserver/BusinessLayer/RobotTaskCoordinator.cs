using System.Collections.Concurrent;
using ApiObject.Cuopt;
using CuOptClientService;
using CuOptClientService.Common;
using DbObject;
using ShareMemoryData;
using VDA5050Message;

namespace BusinessLayer;

/// <summary>
/// Central task pipeline: queue tasks, wait until robots are ready (at last released node),
/// invoke CuOpt, and return VDA5050 orders.
/// </summary>
public class RobotTaskCoordinator
{
    private readonly CuOptClient _cuOptClient;
    private readonly ConcurrentQueue<RobotTask> _taskQueue = new();
    private readonly object _lock = new();
    private Thread? _threadControlRobot;
    public RobotTaskCoordinator(CuOptClient cuOptClient)
    {
        _cuOptClient = cuOptClient;
    }

    public void Enqueue(RobotTask task)
    {
        _taskQueue.Enqueue(task);
    }
    

    /// <summary>
    /// Drain queued tasks and solve when at least one robot is ready (LastNodeReleased == true).
    /// </summary>
    public async Task<Dictionary<string, List<Order>>> TrySolveAsync(
        IEnumerable<RobotStatus> robots,
        Map map,
        CancellationToken ct = default)
    {
        var robotList = robots?.Where(r => r.LastNodeReleased).ToList() ?? new List<RobotStatus>();
        if (robotList.Count == 0 || map.Points == null || map.Routes == null)
        {
            return new Dictionary<string, List<Order>>();
        }

        var tasks = DrainTasks();
        if (tasks.Count == 0)
        {
            return new Dictionary<string, List<Order>>();
        }

        // Solve with CuOpt and map output to orders
        var orders = await _cuOptClient.SolveAsync(robotList, tasks, map.Points, map.Routes, ct).ConfigureAwait(false);
        return orders;
    }

    private List<RobotTask> DrainTasks()
    {
        var result = new List<RobotTask>();
        lock (_lock)
        {
            while (_taskQueue.TryDequeue(out var task))
            {
                result.Add(task);
            }
        }
        return result;
    }
}
