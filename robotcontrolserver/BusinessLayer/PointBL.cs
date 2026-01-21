using BusinessLayer.Interfaces;
using DataAccess.Interface;
using DataAccess.Interfaces;
using DbObject;
using DbObject.Common;
using System.Data;

namespace BusinessLayer
{
    public class PointBL : BaseBL<Point>, IPointBL
    {
        private readonly IBaseDA<PointType> _pointTypeDa;
        public override string TableName => DatabaseEnum.TableName.Point;
        public PointBL(IBaseDA<Point> baseDA, IBaseDA<PointType> pointTypeDa, IDbManagement dbManagement) : base(baseDA, dbManagement)
        {
            _pointTypeDa = pointTypeDa;
        }

        public override int GetChildData(Point data, IDbConnection? connection)
        {
            if (connection != null)
            {
                data.PointType = _pointTypeDa.Query(new PointType { Id = data.PointTypeId }, connection)?.FirstOrDefault();
            }

            return base.GetChildData(data, connection);
        }
    }
}
