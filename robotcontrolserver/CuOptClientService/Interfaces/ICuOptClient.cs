using DbObject;
using ShareMemoryData;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using VDA5050Message;

namespace CuOptClientService.Interfaces
{
    public interface ICuOptClient
    {
        Task<Dictionary<string, List<Order>>> SolveAsync(IEnumerable<RobotStatus> robots, IEnumerable<RobotTask> tasks, IEnumerable<Point> points, IEnumerable<Route> routes, CancellationToken ct = default);
    }
}
