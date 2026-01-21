using BusinessLayer.Interfaces;
using DataAccess.Interface;
using DataAccess.Interfaces;
using DbObject;
using DbObject.Common;
using System.Data;

namespace BusinessLayer
{
    public class RobotTaskBL : BaseBL<RobotTask>, IRobotTaskBL
    {
        private readonly IBaseDA<Point> _pointDa;
        private readonly IBaseDA<RobotType> _robotTypeDa;

        public override string TableName => DatabaseEnum.TableName.RobotTask;
        public RobotTaskBL(IBaseDA<RobotTask> baseDA, IBaseDA<Point> pointDa, IBaseDA<RobotType> robotTypeDa, IDbManagement dbManagement) : base(baseDA, dbManagement)
        {
            _pointDa = pointDa;
            _robotTypeDa = robotTypeDa;
        }

        public override int GetChildData(RobotTask data, IDbConnection? connection)
        {
            if (connection != null)
            {
                if (data.DeliveryFromPointId.HasValue)
                {
                    data.DeliveryFrom = _pointDa.Query(new Point { Id = data.DeliveryFromPointId }, connection)?.FirstOrDefault();
                }
                if (data.DeliveryToPointId.HasValue)
                {
                    data.DeliveryTo = _pointDa.Query(new Point { Id = data.DeliveryToPointId }, connection)?.FirstOrDefault();
                }
                if (data.RobotTypeId.HasValue)
                {
                    data.RobotType = _robotTypeDa.Query(new RobotType { Id = data.RobotTypeId }, connection)?.FirstOrDefault();
                }
            }
            return base.GetChildData(data, connection);
        }
    }
}
