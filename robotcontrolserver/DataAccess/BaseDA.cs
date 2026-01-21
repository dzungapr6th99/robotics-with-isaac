using DataAccess.Extensions;
using DataAccess.Interfaces;
using System.Data;
namespace DataAccess
{


    public class BaseDA<T> : IBaseDA<T> where T : class
    {
        public virtual string TableName { get; } = string.Empty;

        public BaseDA()
        {
        }

        public virtual int Insert(T obj, IDbTransaction transaction)
        {
            if (transaction != null)
            {
                var connection = transaction.Connection;
                if (connection != null)
                {
                    int inserted = connection.Insert<T>(obj, transaction);
                    return inserted;
                }
                else
                {
                    return -1;
                }
            }
            else
            {
                return -1;
            }
        }

        /// <summary>
        /// update Data
        /// </summary>
        /// <param name="newData"></param>
        /// <param name="oldData"></param>
        /// <returns></returns>
        public virtual int Update(T newData, T oldData, IDbTransaction transaction)
        {
            if (transaction != null)
            {
                var connection = transaction.Connection;
                if (connection != null)
                {
                    int updated = connection.Update(newData, oldData, transaction);
                    return updated;

                }
                else
                {
                    return -1;
                }

            }
            else
            {
                return -1;
            }
        }

        public virtual int Delete(T obj, IDbTransaction transaction)
        {
            if (transaction != null)
            {
                var connection = transaction.Connection;
                if (connection != null)
                {

                    var deleted = connection.Delete<T>(obj, transaction);
                    return deleted;
                }
                else
                {
                    return -1;
                }

            }
            else
            {
                return -1;
            }
        }


        /// <summary>
        /// Function get data from db. the input is Dictionary or json object
        /// </summary>
        /// <param name="param">The dictionary. Key is field of table and value is value of the column</param>
        /// <returns></returns>
        public virtual List<T> Query(object? param, IDbConnection connection)
        {

            if (connection != null)
            {
                List<T> getFromDb = connection.Get<T>(param)?.ToList() ?? new List<T>();
                return getFromDb;
            }
            else
            {
                return new List<T>();
            }
        }
    }

}