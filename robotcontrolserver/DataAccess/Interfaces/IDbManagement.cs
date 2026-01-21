using System;
using System.Collections.Generic;
using System.Data;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace DataAccess.Interface
{
    public interface IDbManagement
    {
        public IDbConnection GetConnection();
        //public IDbTransaction GetTransaction();

    }
}
