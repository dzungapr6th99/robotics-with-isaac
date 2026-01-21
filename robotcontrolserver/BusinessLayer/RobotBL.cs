using BusinessLayer.Interfaces;
using DataAccess.Interface;
using DataAccess.Interfaces;
using DbObject;
using DbObject.Common;
using System.Data;

namespace BusinessLayer
{
    public class RobotBL : BaseBL<Robot>, IRobotBL
    {
        private readonly IBaseDA<RobotType> _robotTypeDa;

        public override string TableName => DatabaseEnum.TableName.Robot;
        public RobotBL(IBaseDA<Robot> baseDA, IBaseDA<RobotType> robotTypeDa, IDbManagement dbManagement) : base(baseDA, dbManagement)
        {
            _robotTypeDa = robotTypeDa;
        }

        public override int GetChildData(Robot data, IDbConnection? connection)
        {
            if (connection != null && data.RobotTypeId.HasValue)
            {
                data.RobotType = _robotTypeDa.Query(new RobotType { Id = data.RobotTypeId }, connection)?.FirstOrDefault();
            }
            return base.GetChildData(data, connection);
        }
    }
}
