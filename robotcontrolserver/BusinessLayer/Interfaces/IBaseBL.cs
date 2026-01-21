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
        public bool BeforeInsert(T entity, ref List<int> returnCode, ref List<string> returnMessage);
        public int Update(T entity, out List<int> returnCode, out List<string> returnMessage);
        public int Update(List<T> entities, out List<int> returnCode, out List<string> returnMessage);
        public bool BeforeUpdate(T entity, T oldData, ref List<int> returnCode, ref List<string> returnMessage);
        public int Delete(T entity, out List<int> returnCode, out List<string> returnMessage);
        public int Delete(List<T> entities, out List<int> returnCode, out List<string> returnMessage);
        public int Delete(int id, out List<int> returnCode, out List<string> returnMessage);
        public int Delete(List<int> ids, out List<int> returnCode, out List<string> returnMessage);
        public bool BeforeDelete(T entity, ref List<int> returnCode, ref List<string> returnMessage);
        public T? GetById(int id, out int returnCode, out string returnMessage);
        public List<T>? GetByIds(List<int> Ids, out int returnCode, out string returnMessage);
    }
}
