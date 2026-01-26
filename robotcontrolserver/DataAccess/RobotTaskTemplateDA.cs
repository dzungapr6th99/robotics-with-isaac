using DataAccess.Interfaces;
using DbObject;
using System.Data;

namespace DataAccess
{
    public class RobotTaskTemplateDA : BaseDA<RobotTaskTemplate>, IRobotTaskTemplateDA
    {
        public RobotTaskTemplateDA() : base()
        {
        }

        public override int Insert(RobotTaskTemplate obj, IDbTransaction transaction)
        {
            return base.Insert(obj, transaction);
        }

        public override int Update(RobotTaskTemplate newData, RobotTaskTemplate oldData, IDbTransaction transaction)
        {
            return base.Update(newData, oldData, transaction);
        }

        public override int Delete(RobotTaskTemplate obj, IDbTransaction transaction)
        {
            return base.Delete(obj, transaction);
        }

        public override List<RobotTaskTemplate> Query(object? param, IDbConnection connection)
        {
            return base.Query(param, connection);
        }
    }
}
