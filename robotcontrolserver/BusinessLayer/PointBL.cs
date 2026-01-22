using BusinessLayer.Interfaces;
using CommonLib;
using DataAccess.Interface;
using DataAccess.Interfaces;
using DbObject;
using DbObject.Common;
using System.Data;

namespace BusinessLayer
{
    public class PointBL : BaseBL<Point>, IPointBL
    {
        private readonly IBaseDA<PointType> _pointTypeDA;
        private readonly IBaseDA<Map> _mapDA;
        public override string TableName => DatabaseEnum.TableName.Point;
        public PointBL(IBaseDA<Point> baseDA, IBaseDA<PointType> pointTypeDA, IBaseDA<Map> mapDA,IDbManagement dbManagement) : base(baseDA, dbManagement)
        {
            _pointTypeDA = pointTypeDA;
            _mapDA = mapDA;
        }

        public override int GetChildData(Point data, IDbConnection? connection)
        {
            if (connection != null)
            {
                data.PointType = _pointTypeDA.Query(new PointType { Id = data.PointTypeId }, connection)?.FirstOrDefault();
            }

            return base.GetChildData(data, connection);
        }

        public override bool BeforeInsert(Point entity, IDbConnection connection, ref List<int> returnCode, ref List<string> returnMessage)
        {
            Map? map = _mapDA.Query(new Map()
            {
                Id = entity.MapId
            }, connection)?.FirstOrDefault();
            if ( map == null)
            {
                returnCode.Add(ConstData.ReturnCode.Point.MAP_NOT_EXIST_IN_DATABASE);
                returnMessage.Add(ConstData.ReturnMessage.Point.MAP_NOT_EXIST_IN_DATABASE);
                return false;
            }
            return true;
        }

        public override bool BeforeUpdate(Point entity, Point oldData, IDbConnection connection, ref List<int> returnCode, ref List<string> returnMessage)
        {
            if (entity.MapId != oldData.MapId)
            {
                returnCode.Add(ConstData.ReturnCode.Point.MAP_NOT_EXIST_IN_DATABASE);
                returnMessage.Add(ConstData.ReturnMessage.Point.MAP_NOT_EXIST_IN_DATABASE);
                return false;
            }    
            return true;
        }
    }
}
