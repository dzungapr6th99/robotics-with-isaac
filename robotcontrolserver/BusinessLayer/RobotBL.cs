using BusinessLayer.Interfaces;
using CommonLib;
using DataAccess.Interface;
using DataAccess.Interfaces;
using DbObject;
using DbObject.Common;
using System.Data;

namespace BusinessLayer
{
    public class RobotBL : BaseBL<Robot>, IRobotBL
    {
        private readonly IBaseDA<RobotType> _robotTypeDA;

        public override string TableName => DatabaseEnum.TableName.Robot;
        public RobotBL(IBaseDA<Robot> baseDA, IBaseDA<RobotType> robotTypeDA, IDbManagement dbManagement) : base(baseDA, dbManagement)
        {
            _robotTypeDA = robotTypeDA;
        }

        public override int GetChildData(Robot data, IDbConnection? connection)
        {
            if (connection != null && data.RobotTypeId.HasValue)
            {
                data.RobotType = _robotTypeDA.Query(new RobotType { Id = data.RobotTypeId }, connection)?.FirstOrDefault();
            }
            return base.GetChildData(data, connection);
        }

        public override bool BeforeInsert(Robot entity, IDbConnection connection, ref List<int> returnCode, ref List<string> returnMessage)
        {
            return ValidateRobot(entity, connection, ref returnCode, ref returnMessage);
        }

        public override bool BeforeUpdate(Robot entity, Robot oldData, IDbConnection connection, ref List<int> returnCode, ref List<string> returnMessage)
        {
            return ValidateRobot(entity, connection, ref returnCode, ref returnMessage);
        }

        private bool ValidateRobot(Robot entity, IDbConnection connection, ref List<int> returnCode, ref List<string> returnMessage)
        {
            if (string.IsNullOrWhiteSpace(entity.SerialNumber))
            {
                returnCode.Add(ConstData.ReturnCode.SERVICE_GET_ERROR);
                returnMessage.Add("Serial number must not be empty");
                return false;
            }

            Robot? existed = _baseDA.Query(new Robot { SerialNumber = entity.SerialNumber }, connection)?.FirstOrDefault();
            if (existed != null && existed.Id != entity.Id)
            {
                returnCode.Add(ConstData.ReturnCode.Robot.SERIAL_NUMBER_DUPLICATED);
                returnMessage.Add(ConstData.ReturnMessage.Robot.SERIAL_NUMBER_DUPLICATED);
                return false;
            }

            return true;
        }
    }
}
