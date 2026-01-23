using System.Collections.Concurrent;
using DbObject;
using VDA5050Message;

namespace ShareMemoryData
{
    public static class LocalMemory
    {
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
                    SerialNumber = robot.SerialNumber
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
                robotStatus.CoordinateX = state.AgvPosition?.X ?? 0;
                robotStatus.CoordinateY = state.AgvPosition?.Y ?? 0;
                robotStatus.VelocityX = state.Velocity?.Vx ?? 0;
                robotStatus.VelocityY = state.Velocity?.Vy ?? 0;
            }
        }
    }
}
