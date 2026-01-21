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
    public class RouteDA : BaseDA<Route>, IRouteDA
    {
        public RouteDA() : base()
        {
        }

        public override int Insert(Route obj, IDbTransaction transaction)
        {
            return base.Insert(obj, transaction);
        }

        public override int Update(Route newData, Route oldData, IDbTransaction transaction)
        {
            return base.Update(newData, oldData, transaction);
        }

        public override int Delete(Route obj, IDbTransaction transaction)
        {
            return base.Delete(obj, transaction);
        }

        public override List<Route> Query(object? param, IDbConnection connection)
        {
            return base.Query(param, connection);
        }
    }
}
