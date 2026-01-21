using System;
using System.Collections.Generic;
using System.Data;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace DataAccess.Interfaces
{
    public interface IBaseDA<T> where T : class
    {
        int Insert(T obj, IDbTransaction transaction);
        int Update(T newData, T oldData, IDbTransaction transaction);
        int Delete(T obj, IDbTransaction transaction);
        /// <summary>
        /// Function to insert data to db
        /// </summary>
        /// <param name="param">dictionary or json object. Key is field of DbObject, Value is value of column</param>
        /// <param name="connection"></param>
        /// <param name="isGetChildData">true if want to get child data of object. default is false</param>
        /// <returns></returns>
        List<T> Query(object? param, IDbConnection connection);
    }
}
