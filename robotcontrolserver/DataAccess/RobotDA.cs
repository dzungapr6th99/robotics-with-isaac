using DataAccess.Interfaces;
using DbObject;
using System.Collections.Generic;
using System.Data;

namespace DataAccess
{
    public class RobotDA : BaseDA<Robot>, IRobotDA
    {
        public RobotDA() : base()
        {
        }

        public override int Insert(Robot obj, IDbTransaction transaction)
        {
            return base.Insert(obj, transaction);
        }

        public override int Update(Robot newData, Robot oldData, IDbTransaction transaction)
        {
            return base.Update(newData, oldData, transaction);
        }

        public override int Delete(Robot obj, IDbTransaction transaction)
        {
            return base.Delete(obj, transaction);
        }

        public override List<Robot> Query(object? param, IDbConnection connection)
        {
            return base.Query(param, connection);
        }
    }
}
