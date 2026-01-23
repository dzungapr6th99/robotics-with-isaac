using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using DbMap = DbObject.Map;

namespace BusinessLayer.Interfaces
{
    public interface IMapBL
    {
        DbMap? GetLatest(out int returnCode, out string returnMessage);
        bool ValidateMap(int mapId, out int returnCode, out string returnMessage, out List<string> details);
        bool AssignMapToRobots(int mapId, List<int> robotIds, out int returnCode, out string returnMessage, out List<string> details);
        bool DownloadMap(int mapId, List<int> robotIds, out int returnCode, out string returnMessage, out List<string> details);
    }
}
