using DataAccess.Interfaces;
using DbObject;
using System.Collections.Generic;
using System.Data;

namespace DataAccess
{
    public class RobotTaskDA : BaseDA<RobotTask>, IRobotTaskDA
    {
        public RobotTaskDA() : base()
        {
        }

        public override int Insert(RobotTask obj, IDbTransaction transaction)
        {
            return base.Insert(obj, transaction);
        }

        public override int Update(RobotTask newData, RobotTask oldData, IDbTransaction transaction)
        {
            return base.Update(newData, oldData, transaction);
        }

        public override int Delete(RobotTask obj, IDbTransaction transaction)
        {
            return base.Delete(obj, transaction);
        }

        public override List<RobotTask> Query(object? param, IDbConnection connection)
        {
            return base.Query(param, connection);
        }
    }
}
