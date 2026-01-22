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
        private readonly IBaseDA<Point> _pointDA;
        private readonly IBaseDA<RobotType> _robotTypeDA;

        public override string TableName => DatabaseEnum.TableName.RobotTask;
        public RobotTaskBL(IBaseDA<RobotTask> baseDA, IBaseDA<Point> pointDa, IBaseDA<RobotType> robotTypeDa, IDbManagement dbManagement) : base(baseDA, dbManagement)
        {
            _pointDA = pointDa;
            _robotTypeDA = robotTypeDa;
        }

        public override int GetChildData(RobotTask data, IDbConnection? connection)
        {
            if (connection != null)
            {
                if (data.DeliveryFromPointId.HasValue)
                {
                    data.DeliveryFrom = _pointDA.Query(new Point { Id = data.DeliveryFromPointId }, connection)?.FirstOrDefault();
                }
                if (data.DeliveryToPointId.HasValue)
                {
                    data.DeliveryTo = _pointDA.Query(new Point { Id = data.DeliveryToPointId }, connection)?.FirstOrDefault();
                }
                if (data.RobotTypeId.HasValue)
                {
                    data.RobotType = _robotTypeDA.Query(new RobotType { Id = data.RobotTypeId }, connection)?.FirstOrDefault();
                }
            }
            return base.GetChildData(data, connection);
        }
    }
}
