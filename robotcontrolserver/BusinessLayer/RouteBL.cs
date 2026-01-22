using BusinessLayer.Interfaces;
using CommonLib;
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
        private readonly IBaseDA<Map> _mapDa;

        public override string TableName => DatabaseEnum.TableName.Route;
        public RouteBL(IBaseDA<Route> baseDA, IBaseDA<Point> pointDa, IBaseDA<Map> mapDa, IDbManagement dbManagement) : base(baseDA, dbManagement)
        {
            _pointDa = pointDa;
            _mapDa = mapDa;
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

        public override bool BeforeInsert(Route entity, IDbConnection connection, ref List<int> returnCode, ref List<string> returnMessage)
        {
            return ValidateRoute(entity, connection, ref returnCode, ref returnMessage);
        }

        public override bool BeforeUpdate(Route entity, Route oldData, IDbConnection connection, ref List<int> returnCode, ref List<string> returnMessage)
        {
            return ValidateRoute(entity, connection, ref returnCode, ref returnMessage);
        }

        private bool ValidateRoute(Route entity, IDbConnection connection, ref List<int> returnCode, ref List<string> returnMessage)
        {
            if (!entity.MapId.HasValue || !entity.FromPointId.HasValue || !entity.ToPointId.HasValue)
            {
                returnCode.Add(ConstData.ReturnCode.SERVICE_GET_ERROR);
                returnMessage.Add(ConstData.ReturnMessage.ERROR_WHEN_INSERT_DATA);
                return false;
            }

            Map? map = _mapDa.Query(new Map { Id = entity.MapId }, connection)?.FirstOrDefault();
            if (map == null)
            {
                returnCode.Add(ConstData.ReturnCode.Route.MAP_NOT_EXIST_IN_DATABASE);
                returnMessage.Add(ConstData.ReturnMessage.Route.MAP_NOT_EXIST_IN_DATABASE);
                return false;
            }

            Point? fromPoint = _pointDa.Query(new Point { Id = entity.FromPointId }, connection)?.FirstOrDefault();
            if (fromPoint == null)
            {
                returnCode.Add(ConstData.ReturnCode.Route.FROM_POINT_NOT_EXIST_IN_DATABASE);
                returnMessage.Add(ConstData.ReturnMessage.Route.FROM_POINT_NOT_EXIST_IN_DATABASE);
                return false;
            }

            Point? toPoint = _pointDa.Query(new Point { Id = entity.ToPointId }, connection)?.FirstOrDefault();
            if (toPoint == null)
            {
                returnCode.Add(ConstData.ReturnCode.Route.TO_POINT_NOT_EXIST_IN_DATABASE);
                returnMessage.Add(ConstData.ReturnMessage.Route.TO_POINT_NOT_EXIST_IN_DATABASE);
                return false;
            }

            if (fromPoint.MapId != entity.MapId || toPoint.MapId != entity.MapId)
            {
                returnCode.Add(ConstData.ReturnCode.Route.POINT_NOT_IN_MAP);
                returnMessage.Add(ConstData.ReturnMessage.Route.POINT_NOT_IN_MAP);
                return false;
            }

            return true;
        }
    }
}
