using DataAccess.Interfaces;
using DbObject;
using System.Collections.Generic;
using System.Data;

namespace DataAccess
{
    public class RobotTypeDA : BaseDA<RobotType>, IRobotTypeDA
    {
        public RobotTypeDA() : base()
        {
        }

        public override int Insert(RobotType obj, IDbTransaction transaction)
        {
            return base.Insert(obj, transaction);
        }

        public override int Update(RobotType newData, RobotType oldData, IDbTransaction transaction)
        {
            return base.Update(newData, oldData, transaction);
        }

        public override int Delete(RobotType obj, IDbTransaction transaction)
        {
            return base.Delete(obj, transaction);
        }

        public override List<RobotType> Query(object? param, IDbConnection connection)
        {
            return base.Query(param, connection);
        }
    }
}
