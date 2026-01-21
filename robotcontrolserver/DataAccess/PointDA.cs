using DataAccess.Interfaces;
using DbObject;
using System;
using System.Collections.Generic;
using System.Data;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace DataAccess
{
    public class PointDA : BaseDA<Point>, IPointDA
    {
        public PointDA(): base()
        {

        }
        public override int Insert(Point obj, IDbTransaction transaction)
        {
            return base.Insert(obj, transaction);
        }

        public override int Update(Point newData, Point oldData, IDbTransaction transaction)
        {
            return base.Update(newData, oldData, transaction);
        }

        public override int Delete(Point obj, IDbTransaction transaction)
        {
            return base.Delete(obj, transaction);
        }

        public override List<Point> Query(object? param, IDbConnection connection)
        {
            return base.Query(param, connection);
        }

    }
}
