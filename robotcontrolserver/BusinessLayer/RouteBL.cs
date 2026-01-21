using BusinessLayer.Interfaces;
using DataAccess.Interface;
using DataAccess.Interfaces;
using DbObject;
using DbObject.Common;
using System.Data;

namespace BusinessLayer
{
    public class RouteBL : BaseBL<Route>, IRouteBL
    {
        private readonly IBaseDA<Point> _pointDa;

        public override string TableName => DatabaseEnum.TableName.Route;
        public RouteBL(IBaseDA<Route> baseDA, IBaseDA<Point> pointDa, IDbManagement dbManagement) : base(baseDA, dbManagement)
        {
            _pointDa = pointDa;
        }

        public override int GetChildData(Route data, IDbConnection? connection)
        {
            if (connection != null)
            {
                data.FromPoint = _pointDa.Query(new Point { Id = data.FromPointId }, connection)?.FirstOrDefault();
                data.ToPoint = _pointDa.Query(new Point { Id = data.ToPointId }, connection)?.FirstOrDefault();
            }
            return base.GetChildData(data, connection);
        }
    }
}
