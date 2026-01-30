using System;
using System.Collections.Concurrent;
using System.Linq;
using DbObject;
using VDA5050Message;

namespace ShareMemoryData
{
    public static class LocalMemory
    {
        public static event Action<RobotStatus>? RobotReady;
        public static event Action? RobotIdle;

        private static readonly ConcurrentDictionary<Map, List<RobotStatus>> _mapWithRobotStatus = new();
        private static readonly ConcurrentDictionary<string, RobotStatus> _dictRobotStatus = new();

        public static void AssignMap(Map map)
        {
            _mapWithRobotStatus.TryAdd(map, new List<RobotStatus>());
        }

        public static void AssignRobot(int mapId, Robot robot)
        {
            if (!string.IsNullOrEmpty(robot.SerialNumber))
            {
                RobotStatus robotStatus = new RobotStatus
                {
                    SerialNumber = robot.SerialNumber,
                    MajorVersion = robot.MajorVersion ?? string.Empty,
                    InterfaceName = robot.InterfaceName ?? string.Empty,
                    Manufacturer = robot.Manufacturer ?? string.Empty,
                    RobotTypeId = robot.RobotTypeId ?? 0,
                    
                };
                Map? map = _mapWithRobotStatus.Keys.FirstOrDefault(x => x.Id == mapId);
                if (map != null)
                {
                    _mapWithRobotStatus[map].Add(robotStatus);
                    _dictRobotStatus.TryAdd(robot.SerialNumber, robotStatus);
                }
            }
        }

        public static void UpdateRobotStatus(State state)
        {
            if (state == null || string.IsNullOrEmpty(state.SerialNumber))
            {
                return;
            }
            if (_dictRobotStatus.TryGetValue(state.SerialNumber, out var robotStatus))
            {
                var wasReleased = robotStatus.LastNodeReleased;
                var wasBusy = !string.IsNullOrEmpty(robotStatus.DoingTask);
                robotStatus.CoordinateX = state.AgvPosition?.X ?? 0;
                robotStatus.CoordinateY = state.AgvPosition?.Y ?? 0;
                robotStatus.VelocityX = state.Velocity?.Vx ?? 0;
                robotStatus.VelocityY = state.Velocity?.Vy ?? 0;
                robotStatus.LastNodeId = state.LastNodeId ?? string.Empty;
                robotStatus.LastNodeReleased = state.NodeStates?.Any(n => n.Released && n.NodeId == robotStatus.LastNodeId) == true;
                robotStatus.DoingTask = state.Driving ? (state.OrderId ?? string.Empty) : string.Empty;

                if (!wasReleased && robotStatus.LastNodeReleased)
                {
                    RobotReady?.Invoke(robotStatus);
                }

                var isIdle = string.IsNullOrEmpty(robotStatus.DoingTask);
                if (!wasBusy && isIdle)
                {
                    RobotIdle?.Invoke();
                }
            }
        }

        public static bool HaveRobotIdle(out Dictionary<Map, List<RobotStatus>> _dictMapRobotStatus)
        {
            bool haveRobotIdle = false;
            _dictMapRobotStatus = new Dictionary<Map, List<RobotStatus>>();
            foreach (var keyValuePair in _mapWithRobotStatus)
            {
                var listRobotStatus = new List<RobotStatus>();
                foreach (RobotStatus robotStatus in keyValuePair.Value)
                {
                    if (string.IsNullOrEmpty(robotStatus.DoingTask))
                    {
                        listRobotStatus.Add(robotStatus);
                    }
                }
                if (listRobotStatus.Count() > 0)
                {
                    _dictMapRobotStatus.Add(keyValuePair.Key, listRobotStatus);
                    haveRobotIdle = true;
                }
            }
            return haveRobotIdle;
        }
    }
}
