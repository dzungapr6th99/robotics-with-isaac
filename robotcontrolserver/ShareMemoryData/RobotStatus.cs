using System.Collections.Concurrent;
using System.Net.NetworkInformation;
using DbObject;
using VDA5050Message;

namespace ShareMemoryData
{

    public static class LocalMemory
    {
        private static ConcurrentDictionary<Map, List<RobotStatus>> _mapWithRobotStatus = new ConcurrentDictionary<Map, List<RobotStatus>>();
        private static ConcurrentDictionary<string ,RobotStatus> _dictRobotStatus = new ConcurrentDictionary<string, RobotStatus>();

        public static void AssignMap(Map map)
        {
            _mapWithRobotStatus.TryAdd(map, new List<RobotStatus>());
        }

        public static void AssignRobot(int mapId, Robot robot)
        {
            if (!string.IsNullOrEmpty(robot.SerialNumber))
            {
                RobotStatus robotStatus = new RobotStatus()
                {
                    SerialNumber = robot.SerialNumber
                };
                Map? map = _mapWithRobotStatus.Keys.Where(x => x.Id == mapId)?.FirstOrDefault();
                if (map != null)
                {
                    _mapWithRobotStatus[map].Add(robotStatus);
                    _dictRobotStatus.TryAdd(robot.SerialNumber ,robotStatus);
                }
            }
        }

        public static void UpdateRobotStatus(State state)
        {
            RobotStatus robotStatus = _dictRobotStatus[state.SerialNumber];
            robotStatus.CoordinateX = state.AgvPosition?.X ?? 0;
            robotStatus.CoordinateY = state.AgvPosition?.Y ?? 0;
            robotStatus.VelocityX = state.Velocity?.Vx ?? 0 ;
            robotStatus.VelocityY = state.Velocity?.Vy ?? 0;
            
        }
    }

}
