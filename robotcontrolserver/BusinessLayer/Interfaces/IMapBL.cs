using System;
using System.Collections.Generic;
using DbMap = DbObject.Map;
using DbObject;
using ApiObject;

namespace BusinessLayer.Interfaces
{
    public interface IMapBL : IBaseBL<DbMap>
    {
        DbMap? GetLatest(out int returnCode, out string returnMessage);
        bool ValidateMap(int mapId, out int returnCode, out string returnMessage, out List<string> details);
        bool AssignMapToRobots(int mapId, List<int> robotIds, out int returnCode, out string returnMessage, out List<string> details);
        bool DownloadMap(int mapId, List<int> robotIds, out int returnCode, out string returnMessage, out List<string> details);

        bool ImportMatrix(DbMap map, ImportMatrix matrix, out string returnMessage);
    }
}
