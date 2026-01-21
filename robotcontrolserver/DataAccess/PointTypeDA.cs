using DataAccess.Interfaces;
using DbObject;
using System.Collections.Generic;
using System.Data;

namespace DataAccess
{
    public class PointTypeDA : BaseDA<PointType>, IPointTypeDA
    {
        public PointTypeDA() : base()
        {
        }

        public override int Insert(PointType obj, IDbTransaction transaction)
        {
            return base.Insert(obj, transaction);
        }

        public override int Update(PointType newData, PointType oldData, IDbTransaction transaction)
        {
            return base.Update(newData, oldData, transaction);
        }

        public override int Delete(PointType obj, IDbTransaction transaction)
        {
            return base.Delete(obj, transaction);
        }

        public override List<PointType> Query(object? param, IDbConnection connection)
        {
            return base.Query(param, connection);
        }
    }
}
