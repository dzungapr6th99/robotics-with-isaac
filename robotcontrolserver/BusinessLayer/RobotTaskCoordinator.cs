using System;
using System.Collections.Concurrent;
using ApiObject.Cuopt;
using BusinessLayer.Interfaces;
using CuOptClientService;
using CuOptClientService.Common;
using CuOptClientService.Interfaces;
using DbObject;
using RobotControl.Interfaces;
using ShareMemoryData;
using VDA5050Message;

namespace BusinessLayer;

/// <summary>
/// Central task pipeline: queue tasks, wait until robots are ready (at last released node),
/// invoke CuOpt, and return VDA5050 orders.
/// </summary>
public class RobotTaskCoordinator : IRobotTaskCoordinator
{
    private readonly ICuOptClient _cuOptClient;
    private readonly ConcurrentQueue<RobotTask> _taskQueue = new();
    private readonly ConcurrentDictionary<string, ConcurrentQueue<Order>> _pendingOrders = new();
    private readonly object _lock = new();
    public event Action<string, Order>? OnOrderReadyToSend;
    private readonly IAgvControl _agvControl;

    public RobotTaskCoordinator(ICuOptClient cuOptClient, IAgvControl agvControl)
    {
        _cuOptClient = cuOptClient;
        _agvControl = agvControl;
        LocalMemory.RobotReady += HandleRobotReady;
        LocalMemory.RobotIdle += HandleRobotIdle;
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

        // store pending orders for later dispatch
        foreach (var kvp in orders)
        {
            var queue = _pendingOrders.GetOrAdd(kvp.Key, _ => new ConcurrentQueue<Order>());
            foreach (var order in kvp.Value)
            {
                queue.Enqueue(order);
            }
        }

        // immediately dispatch to any ready robots
        foreach (var robot in robotList)
        {
            HandleRobotReady(robot);
        }

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

    private void HandleRobotReady(RobotStatus robotStatus)
    {
        if (robotStatus == null || string.IsNullOrEmpty(robotStatus.SerialNumber))
            return;

        if (_pendingOrders.TryGetValue(robotStatus.SerialNumber, out var queue))
        {
            if (queue.TryDequeue(out var nextOrder))
            {
                _ = SendOrderAsync(robotStatus, nextOrder);
            }
        }
    }

    private void HandleRobotIdle()
    {
        if (!LocalMemory.HaveRobotIdle(out var mapRobots) || mapRobots.Count == 0)
        {
            return;
        }

        foreach (var kv in mapRobots)
        {
            _ = TrySolveAsync(kv.Value, kv.Key);
        }
    }

    private async Task SendOrderAsync(RobotStatus robotStatus, Order order)
    {
        try
        {
            await _agvControl.SendOrder(order, robotStatus);
            OnOrderReadyToSend?.Invoke(robotStatus.SerialNumber, order);
        }
        catch
        {
            var queue = _pendingOrders.GetOrAdd(robotStatus.SerialNumber, _ => new ConcurrentQueue<Order>());
            queue.Enqueue(order);
        }
    }


}
