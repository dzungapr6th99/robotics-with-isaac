using ApiObject;
using BusinessLayer.Interfaces;
using CommonLib;
using DataAccess.Interface;
using DataAccess.Interfaces;
using DbObject;
using DbObject.Common;
using NLog.Time;
using RobotControl.Interfaces;
using ShareMemoryData;
using System.Data;
using System.Net.WebSockets;
using System.Xml.Linq;
using VDA5050Message;
using VDA5050Message.Base;
using DbMap = DbObject.Map;

namespace BusinessLayer
{
    public class MapBL : BaseBL<DbMap>, IMapBL
    {
        private readonly IBaseDA<Route> _routeBaseDA;
        private readonly IBaseDA<Point> _pointBaseDA;
        private readonly IBaseDA<Robot> _robotBaseDA;
        private readonly IAgvControl _agvControl;
        public override string TableName => DatabaseEnum.TableName.Map;
        public override string TableId => base.TableId;
        public MapBL(IBaseDA<DbMap> baseDA, IBaseDA<Route> routeBaseDA, IBaseDA<Point> pointBaseDA, IBaseDA<Robot> robotBaseDA, IAgvControl agvControl, IDbManagement dbManagement) : base(baseDA, dbManagement)
        {
            _routeBaseDA = routeBaseDA;
            _pointBaseDA = pointBaseDA;
            _robotBaseDA = robotBaseDA;
            _agvControl = agvControl;
        }

        public override int GetChildData(DbMap data, IDbConnection? connection)
        {
            if (connection != null)
            {
                data.Routes = _routeBaseDA.Query(new Route()
                {
                    MapId = data.Id
                }, connection)?.ToList();
                data.Points = _pointBaseDA.Query(new Point()
                {
                    MapId = data.Id
                }, connection)?.ToList();
                foreach (var route in data.Routes)
                {
                    route.FromPoint = _pointBaseDA.Query(new Point()
                    {
                        Id = route.FromPointId
                    }, connection)?.FirstOrDefault();
                    route.ToPoint = _pointBaseDA.Query(new Point()
                    {
                        Id = route.ToPointId
                    }, connection)?.FirstOrDefault();
                }
            }
            return base.GetChildData(data, connection);
        }

        public DbMap? GetLatest(out int returnCode, out string returnMessage)
        {
            returnCode = ConstData.ReturnCode.SUCCESS;
            returnMessage = ConstData.ReturnMessage.SUCCESS;
            try
            {
                using (var connection = _dbManagement.GetConnection())
                {
                    var latest = _baseDA.Query(null, connection)?.OrderByDescending(x => x.Id)?.FirstOrDefault();
                    if (latest != null)
                    {
                        GetChildData(latest, connection);
                    }
                    else
                    {
                        returnCode = ConstData.ReturnCode.SERVICE_GET_ERROR;
                        returnMessage = ConstData.ReturnMessage.ERROR_WHEN_SEARCH_DATA;
                    }
                    return latest;
                }
            }
            catch (Exception ex)
            {
                CommonLib.CommonLog.logDb.Error(ex);
                returnCode = ConstData.ReturnCode.SERVICE_GET_ERROR;
                returnMessage = ConstData.ReturnMessage.SERVICE_GET_ERROR;
                return null;
            }
        }

        public bool ValidateMap(int mapId, out int returnCode, out string returnMessage, out List<string> details)
        {
            details = new List<string>();
            returnCode = ConstData.ReturnCode.SUCCESS;
            returnMessage = ConstData.ReturnMessage.SUCCESS;
            try
            {
                DbMap? map = GetById(mapId, out returnCode, out returnMessage);
                if (map == null)
                {
                    returnMessage = "Map not found";
                    return false;
                }

                if (map.Points == null || map.Points.Count == 0)
                {
                    details.Add("Map does not contain any points.");
                }
                if (map.Routes == null || map.Routes.Count == 0)
                {
                    details.Add("Map does not contain any routes.");
                }
                return details.Count == 0;
            }
            catch (Exception ex)
            {
                CommonLib.CommonLog.logDb.Error(ex);
                returnCode = ConstData.ReturnCode.SERVICE_GET_ERROR;
                returnMessage = ConstData.ReturnMessage.SERVICE_GET_ERROR;
                return false;
            }
        }

        public bool AssignMapToRobots(int mapId, List<int> robotIds, out int returnCode, out string returnMessage, out List<string> details)
        {
            details = new List<string>();
            returnCode = ConstData.ReturnCode.SUCCESS;
            returnMessage = ConstData.ReturnMessage.SUCCESS;
            try
            {
                using (var connection = _dbManagement.GetConnection())
                {
                    DbMap? map = GetById(mapId, out returnCode, out returnMessage);
                    if (map == null)
                    {
                        returnCode = ConstData.ReturnCode.SERVICE_GET_ERROR;
                        returnMessage = "Map not found";
                        return false;
                    }
                    int mapIdValue = map.Id ?? 0;
                    LocalMemory.AssignMap(map);
                    foreach (var robotId in robotIds)
                    {
                        Robot? robot = _robotBaseDA.Query(new Robot { Id = robotId }, connection)?.FirstOrDefault();
                        if (robot == null)
                        {
                            details.Add($"Robot id {robotId} not found.");
                            continue;
                        }
                        LocalMemory.AssignRobot(mapIdValue, robot);
                    }
                }
                return details.Count == 0;
            }
            catch (Exception ex)
            {
                CommonLib.CommonLog.logDb.Error(ex);
                returnCode = ConstData.ReturnCode.SERVICE_GET_ERROR;
                returnMessage = ConstData.ReturnMessage.SERVICE_GET_ERROR;
                return false;
            }
        }

        public bool DownloadMap(int mapId, List<int> robotIds, out int returnCode, out string returnMessage, out List<string> details)
        {
            details = new List<string>();
            returnCode = ConstData.ReturnCode.SUCCESS;
            returnMessage = ConstData.ReturnMessage.SUCCESS;
            try
            {
                DbMap? map = GetById(mapId, out returnCode, out returnMessage);
                if (map == null)
                {
                    returnMessage = "Map not found";
                    returnCode = ConstData.ReturnCode.SERVICE_GET_ERROR;
                    return false;
                }
                if (string.IsNullOrWhiteSpace(map.MinioUrl))
                {
                    details.Add("Map does not have download url");
                    return false;
                }

                using (var connection = _dbManagement.GetConnection())
                {
                    foreach (var robotId in robotIds)
                    {
                        Robot? robot = _robotBaseDA.Query(new Robot { Id = robotId }, connection)?.FirstOrDefault();
                        if (robot == null)
                        {
                            details.Add($"Robot id {robotId} not found.");
                            continue;
                        }
                        var downloadToken = MapDownloadTokenStore.Create(map.Id ?? 0, map.MinioUrl ?? string.Empty, robotId);
                        var status = new RobotStatus
                        {
                            SerialNumber = robot.SerialNumber ?? string.Empty,
                            InterfaceName = robot.InterfaceName ?? ConfigApp.ConfigData.MqttClientConfig.InterfaceName,
                            MajorVersion = ConfigApp.ConfigData.MqttClientConfig.MajorVersion,
                            Manufacturer = robot.Manufacturer ?? ConfigApp.ConfigData.MqttClientConfig.Manufacturer
                        };
                        var actions = new InstantActions
                        {
                            HeaderId = map.Id ?? 0,
                            Timestamp = DateTime.UtcNow,
                            Version = ConfigApp.ConfigData.Version,
                            Manufacturer = status.Manufacturer,
                            SerialNumber = status.SerialNumber,
                            Actions = new List<InstantAction>
                            {
                                new InstantAction
                                {
                                    ActionId = $"downloadMap-{map.Id}",
                                    ActionType = "downloadMap",
                                    BlockingType = BlockingType.SOFT,
                                    ActionParameters = new List<ActionParameter>
                                    {
                                        new ActionParameter{ Key = "token", Value = downloadToken.Token.ToString() },
                                        new ActionParameter{ Key = "mapId", Value = map.Id ?? 0 }
                                    }
                                }
                            }
                        };
                        var sent = _agvControl.SendInstantActions(actions, status).GetAwaiter().GetResult();
                        if (!sent)
                        {
                            details.Add($"Send download map to robot {status.SerialNumber} failed.");
                        }
                    }
                }

                return details.Count == 0;
            }
            catch (Exception ex)
            {
                CommonLog.logDb.Error(ex);
                returnCode = ConstData.ReturnCode.SERVICE_GET_ERROR;
                returnMessage = ConstData.ReturnMessage.SERVICE_GET_ERROR;
                return false;
            }
        }

        public bool ImportMatrix(DbMap map, ImportMatrix matrix, out string returnMessage)
        {
            try
            {
                map.Points = new List<Point>();
                returnMessage = "Success";
                using (var connection = _dbManagement.GetConnection())
                {
                    using var transaction = connection.BeginTransaction();
                    foreach (var node in matrix.Nodes)
                    {
                        Point point = new Point()
                        {
                            Name = $"Node_{node.Id}",
                            MapId = map.Id,
                            X = node.X,
                            Y = node.Y,
                            CreateAt = DateTime.Now,

                        };
                        int nodeId = _pointBaseDA.Insert(point, transaction);
                        if (nodeId == -1)
                        {
                            returnMessage = $"Error when insert Node {node.Id}";
                            transaction.Rollback();
                            return false;
                        }
                        point.Id = nodeId;
                        map.Points.Add(point);
                    }
                    foreach (var edge in matrix.Edges)
                    {
                        int FromPointId = map.Points.Where(x => x.Name == $"Node_{edge.From}").FirstOrDefault()?.Id ?? -1;
                        int ToPointId = map.Points.Where(x => x.Name == $"Node_{edge.To}").FirstOrDefault()?.Id ?? -1;
                        if (FromPointId == -1 || ToPointId == -1)
                        {
                            returnMessage = $"Error because there no Node_{edge.From} or Node_{edge.To}";
                            transaction.Rollback();
                            return false;
                        }
                        Route route = new Route()
                        {
                            MapId = map.Id,
                            FromPointId = FromPointId,
                            ToPointId = ToPointId,
                            Name = $"Edge_{FromPointId}_{ToPointId}",
                            CreateAt = DateTime.Now
                        };
                        int edgeId = _routeBaseDA.Insert(route, transaction);
                        if (edgeId == -1)
                        {
                            returnMessage = $"Error when insert Edge {route.Name}";
                            transaction.Rollback();
                            return false;
                        }
                    }
                    transaction.Commit();
                    return true;
                }
            }
            catch (Exception ex)
            {
                CommonLog.log.Error(ex);
                returnMessage = ex.Message;
                return false;
            }
        }
    }
}
