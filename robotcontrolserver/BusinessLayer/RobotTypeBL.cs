using BusinessLayer.Interfaces;
using DataAccess.Interface;
using DataAccess.Interfaces;
using DbObject;
using DbObject.Common;

namespace BusinessLayer
{
    public class RobotTypeBL : BaseBL<RobotType>, IRobotTypeBL
    {
        public override string TableName => DatabaseEnum.TableName.RobotType;
        public RobotTypeBL(IBaseDA<RobotType> baseDA, IDbManagement dbManagement) : base(baseDA, dbManagement)
        {
        }
    }
}
