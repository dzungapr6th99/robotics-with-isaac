using DbObject;
using System;
using System.Collections.Generic;
using System.Data;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace BusinessLayer.Interfaces
{
    public interface IBaseBL<T> where T : BaseDbObject
    {
        public int Insert(T entity, out List<int> returnCode, out List<string> returnMessage);
        public int Insert(List<T> entities, out List<int> returnCode, out List<string> returnMessage);
        public int InsertChildData(T entity, IDbTransaction? transaction);
        public bool BeforeInsert(T entity, IDbConnection connection, ref List<int> returnCode, ref List<string> returnMessage);
        public int Update(T entity, out List<int> returnCode, out List<string> returnMessage);
        public int Update(List<T> entities, out List<int> returnCode, out List<string> returnMessage);
        public int UpdateChildData(T newData, T oldData, IDbTransaction? transaction);
        public bool BeforeUpdate(T entity, T oldData, IDbConnection connection, ref List<int> returnCode, ref List<string> returnMessage);
        public int Delete(T entity, out List<int> returnCode, out List<string> returnMessage);
        public int Delete(List<T> entities, out List<int> returnCode, out List<string> returnMessage);
        public int Delete(int id, out List<int> returnCode, out List<string> returnMessage);
        public int Delete(List<int> ids, out List<int> returnCode, out List<string> returnMessage);
        public int DeleteChildData (T entity, IDbTransaction? transaction);
        public bool BeforeDelete(T entity, IDbConnection connection, ref List<int> returnCode, ref List<string> returnMessage);
        public T? GetById(int id, out int returnCode, out string returnMessage);
        public List<T>? GetByIds(List<int> Ids, out int returnCode, out string returnMessage);
    }
}
