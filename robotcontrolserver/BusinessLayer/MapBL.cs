using BusinessLayer.Interfaces;
using DataAccess.Interface;
using DataAccess.Interfaces;
using DbObject;
using DbObject.Common;
using System.Data;

namespace BusinessLayer
{
    public class MapBL : BaseBL<Map>, IMapBL
    {
        private readonly IBaseDA<Route> _routeBaseDA;
        private readonly IBaseDA<Point> _pointBaseDA;
        public override string TableName => DatabaseEnum.TableName.Map;
        public override string TableId => base.TableId;
        public MapBL(IBaseDA<Map> baseDA, IBaseDA<Route> routeBaseDA, IBaseDA<Point> pointBaseDA, IDbManagement dbManagement) : base(baseDA, dbManagement)
        {
            _routeBaseDA = routeBaseDA;
            _pointBaseDA = pointBaseDA;
        }

        public override int GetChildData(Map data, IDbConnection? connection)
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
    }
}
