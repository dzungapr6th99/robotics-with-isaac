using BusinessLayer.Interfaces;
using CommonLib;
using DataAccess.Interface;
using DataAccess.Interfaces;
using DbObject;
using DbObject.Extension;
using NLog;
using System.Data;
namespace BusinessLayer
{
    public abstract class BaseBL<T> : IBaseBL<T> where T : BaseDbObject
    {

        public readonly IBaseDA<T> _baseDA;
        public virtual string TableName { get; } = string.Empty;
        public virtual string TableId { get; } = nameof(BaseDbObject.Id);
        public virtual string ObjChange { get; } = string.Empty;
        public readonly IDbManagement _dbManagement;
        public BaseBL(IBaseDA<T> baseDA, IDbManagement dbManagement)
        {
            _baseDA = baseDA;
            _dbManagement = dbManagement;
        }

        public virtual int Insert(T entity, out List<int> returnCode, out List<string> returnMessage)
        {
            returnCode = new List<int>();
            returnMessage = new List<string>();
            int result = ConstData.ReturnCode.SUCCESS;
            try
            {
                bool checkBeforeInsert = BeforeInsert(entity, ref returnCode, ref returnMessage);
                if (checkBeforeInsert)
                {
                    using (var transaction = _dbManagement.GetConnection().BeginTransaction())
                    {
                        int insert = _baseDA.Insert(entity, transaction);
                        int insertChilData = InsertChildData(entity, transaction);

                        if (insert > 0 && insertChilData > 0)
                        {
                            transaction.Commit();
                            returnCode.Add(ConstData.ReturnCode.SUCCESS);
                            returnMessage.Add(ConstData.ReturnMessage.SUCCESS);
                        }
                        else
                        {
                            returnCode.Add(ConstData.ReturnCode.SERVICE_GET_ERROR);
                            returnMessage.Add(ConstData.ReturnMessage.ERROR_WHEN_INSERT_DATA);
                            transaction.Rollback();
                        }
                        result = insert;
                    }
                }
                else
                {
                    result = -1;
                }
                return result;
            }
            catch (Exception ex)
            {
                CommonLog.logDb.Error(ex);
                returnCode.Add(ConstData.ReturnCode.SERVICE_GET_ERROR);
                returnMessage.Add(ConstData.ReturnMessage.SERVICE_GET_ERROR + $@" [{ex.Message}]");
                return -1;
            }

        }

        public virtual int Insert(List<T> entities, out List<int> returnCode, out List<string> returnMessage)
        {
            returnCode = new List<int>();
            returnMessage = new List<string>();
            try
            {
                bool checkBeforeInsert = false;
                foreach (T entity in entities)
                {
                    checkBeforeInsert = BeforeInsert(entity, ref returnCode, ref returnMessage);
                    if (!checkBeforeInsert)
                    {
                        return -1;
                    }
                }
                using (var transaction = _dbManagement.GetConnection().BeginTransaction())
                {
                    int inserted = 0;
                    for (int i = 0; i < entities.Count; i++)
                    {
                        inserted += _baseDA.Insert(entities[i], transaction);
                    }
                    if (inserted > entities.Count)
                    {
                        returnCode.Add(ConstData.ReturnCode.SERVICE_GET_ERROR);
                        returnMessage.Add(ConstData.ReturnMessage.ERROR_WHEN_INSERT_DATA);
                        transaction.Rollback();
                        return -1;
                    }
                    else
                    {
                        returnCode.Add(ConstData.ReturnCode.SUCCESS);
                        returnMessage.Add(ConstData.ReturnMessage.SUCCESS);
                        transaction.Commit();
                        return inserted;
                    }
                }
            }
            catch (Exception ex)
            {
                CommonLog.logDb.Error(ex);
                returnCode.Add(ConstData.ReturnCode.SERVICE_GET_ERROR);
                returnMessage.Add(ConstData.ReturnMessage.SERVICE_GET_ERROR + $@" [{ex.Message}]");
                return -1;
            }
        }
        public virtual int Update(T entity, out List<int> returnCode, out List<string> returnMessage)
        {
            returnCode = new List<int>();
            returnMessage = new List<string>();
            int result = 0;
            if (entity.Id == null)
            {

                return -1;
            }    
            try
            {
                T? oldData = null;
                using (var connection = _dbManagement.GetConnection())
                {
                    oldData = GetById((int)entity.Id, out int code, out string message);
                    if (oldData == null)
                    {
                        CommonLog.logDb.Warn("Cannot find data in database");
                        returnCode.Add(code);
                        returnMessage.Add(message);
                        return ConstData.ReturnCode.SERVICE_GET_ERROR;
                    }
                    bool checkBeforeUpdate = BeforeUpdate(entity, oldData, ref returnCode, ref returnMessage);
                    if (checkBeforeUpdate)
                    {
                        using (var transaction = connection.BeginTransaction())
                        {


                            int update = _baseDA.Update(entity, oldData, transaction);
                            int updateChildData = UpdateChildData(entity, oldData, transaction);
                            if (update > 0 && updateChildData > 0)
                            {
                                transaction.Commit();
                                returnCode.Add(ConstData.ReturnCode.SUCCESS);
                                returnMessage.Add(ConstData.ReturnMessage.SUCCESS);
                            }
                            else
                            {
                                returnCode.Add(ConstData.ReturnCode.SERVICE_GET_ERROR);
                                returnMessage.Add(ConstData.ReturnMessage.ERROR_WHEN_INSERT_DATA);
                                transaction.Rollback();
                            }
                            result = update;
                        }

                    }
                    else
                    {
                        return -1;
                    }
                }
                return result;

            }
            catch (Exception ex)
            {
                CommonLog.logDb.Error(ex);
                returnCode.Add(ConstData.ReturnCode.SERVICE_GET_ERROR);
                returnMessage.Add(ConstData.ReturnMessage.SERVICE_GET_ERROR + $@" [{ex.Message}]");
                return -1;
            }
        }
        public virtual int Update(List<T> entities, out List<int> returnCode, out List<string> returnMessage)
        {
            returnCode = new List<int>();
            returnMessage = new List<string>();
            try
            {
                using (var connection = _dbManagement.GetConnection())
                {
                    using (var transaction = connection.BeginTransaction())
                    {
                        foreach (var entity in entities)
                        {
                            T? oldData = _baseDA.Query(entity, connection)?.FirstOrDefault();
                            if (oldData == null)
                            {
                                CommonLog.logDb.Warn("Cannot find data in database");
                                returnCode.Add(ConstData.ReturnCode.SERVICE_GET_ERROR);
                                returnMessage.Add(ConstData.ReturnMessage.ERROR_WHEN_UPDATE_DATA);
                                transaction.Rollback();
                                return ConstData.ReturnCode.SERVICE_GET_ERROR;
                            }

                            if (!BeforeUpdate(entity, oldData, ref returnCode, ref returnMessage))
                            {
                                return -1;
                            }

                            int update = _baseDA.Update(entity, oldData, transaction);
                            if (update < 0)
                            {
                                returnCode.Add(ConstData.ReturnCode.SERVICE_GET_ERROR);
                                returnMessage.Add(ConstData.ReturnMessage.ERROR_WHEN_INSERT_DATA);
                                transaction.Rollback();
                                return update;
                            }
                        }

                        transaction.Commit();
                        returnCode.Add(ConstData.ReturnCode.SUCCESS);
                        returnMessage.Add(ConstData.ReturnMessage.SUCCESS);
                        return ConstData.ReturnCode.SUCCESS;
                    }
                }
            }
            catch (Exception ex)
            {
                CommonLog.logDb.Error(ex);
                returnCode.Add(ConstData.ReturnCode.SERVICE_GET_ERROR);
                returnMessage.Add(ConstData.ReturnMessage.SERVICE_GET_ERROR + $@" [{ex.Message}]");
                return -1;
            }
        }
        public virtual int Delete(T entity, out List<int> returnCode, out List<string> returnMessage)
        {
            returnCode = new List<int>();
            returnMessage = new List<string>();
            try
            {
                bool checkBeforeUpdate = BeforeDelete(entity, ref returnCode, ref returnMessage);
                if (checkBeforeUpdate)
                {
                    using (var transaction = _dbManagement.GetConnection().BeginTransaction())
                    {
                        int deleted = _baseDA.Delete(entity, transaction);
                        int deleteChildData = DeleteChildData(entity, transaction);
                        if (deleted > 0 && deleteChildData > 0)
                        {
                            transaction.Commit();
                            returnCode.Add(ConstData.ReturnCode.SUCCESS);
                            returnMessage.Add(ConstData.ReturnMessage.SUCCESS);
                        }
                        else
                        {
                            returnCode.Add(ConstData.ReturnCode.SERVICE_GET_ERROR);
                            returnMessage.Add(ConstData.ReturnMessage.ERROR_WHEN_INSERT_DATA);
                            transaction.Rollback();
                            return deleted;
                        }
                    }
                    return ConstData.ReturnCode.SUCCESS;
                }
                else
                {
                    return -1;
                }
            }
            catch (Exception ex)
            {
                CommonLog.logDb.Error(ex);
                returnCode.Add(ConstData.ReturnCode.SERVICE_GET_ERROR);
                returnMessage.Add(ConstData.ReturnMessage.SERVICE_GET_ERROR + $@" [{ex.Message}]");
                return -1;
            }
        }
        public virtual int Delete(List<T> entities, out List<int> returnCode, out List<string> returnMessage)
        {
            returnCode = new List<int>();
            returnMessage = new List<string>();
            try
            {
                bool checkBeforeDelete = false;
                foreach (T entity in entities)
                {
                    checkBeforeDelete = BeforeDelete(entity, ref returnCode, ref returnMessage);
                    if (!checkBeforeDelete)
                    {
                        return -1;
                    }
                }
                using (var transaction = _dbManagement.GetConnection().BeginTransaction())
                {
                    int deleted = 0;
                    for (int i = 0; i < entities.Count; i++)
                    {
                        deleted += _baseDA.Delete(entities[i], transaction);
                    }
                    if (deleted > entities.Count)
                    {
                        returnCode.Add(ConstData.ReturnCode.SERVICE_GET_ERROR);
                        returnMessage.Add(ConstData.ReturnMessage.ERROR_WHEN_INSERT_DATA);
                        transaction.Rollback();
                        return -1;
                    }
                    else
                    {
                        returnCode.Add(ConstData.ReturnCode.SUCCESS);
                        returnMessage.Add(ConstData.ReturnMessage.SUCCESS);
                        transaction.Commit();
                        return deleted;
                    }
                }

            }
            catch (Exception ex)
            {
                CommonLog.logDb.Error(ex);
                returnCode.Add(ConstData.ReturnCode.SERVICE_GET_ERROR);
                returnMessage.Add(ConstData.ReturnMessage.SERVICE_GET_ERROR + $@" [{ex.Message}]");
                return -1;
            }
        }
        public virtual int Delete(int Id, out List<int> returnCode, out List<string> returnMessage)
        {
            returnCode = new List<int>();
            returnMessage = new List<string>();
            returnCode.Add(ConstData.ReturnCode.SERVICE_GET_ERROR);
            returnMessage.Add(ConstData.ReturnMessage.SERVICE_GET_ERROR);
            return -1;
        }
        public virtual int Delete(List<int> Ids, out List<int> returnCode, out List<string> returnMessage)
        {
            returnCode = new List<int>();
            returnMessage = new List<string>();
            returnCode.Add(ConstData.ReturnCode.SERVICE_GET_ERROR);
            returnMessage.Add(ConstData.ReturnMessage.SERVICE_GET_ERROR);
            return -1;
        }
        public virtual T? GetById(int id, out int returnCode, out string returnMessage)
        {
            try
            {
                returnCode = ConstData.ReturnCode.SUCCESS;
                returnMessage = ConstData.ReturnMessage.SUCCESS;
                using (var connection = _dbManagement.GetConnection())
                {
                    T? data = _baseDA.Query(new Dictionary<string, object>()
                        {
                            {TableId, id}
                        }, connection).FirstOrDefault();
                    if (data != null)
                    {
                        GetChildData(data, connection);
                    }
                    else
                    {
                        returnCode = ConstData.ReturnCode.SERVICE_GET_ERROR;
                        returnMessage = ConstData.ReturnMessage.ERROR_WHEN_SEARCH_DATA;
                    }
                    return data;
                }
            }
            catch (Exception ex)
            {
                CommonLog.logDb.Error(ex);
                returnCode = ConstData.ReturnCode.SERVICE_GET_ERROR;
                returnMessage = ConstData.ReturnMessage.SERVICE_GET_ERROR;
                return null;
            }
        }

       
        public List<T>? GetByIds(List<int> Ids, out int returnCode, out string returnMessage)
        {
            returnCode = ConstData.ReturnCode.SUCCESS;
            returnMessage = ConstData.ReturnMessage.SUCCESS;
            List<T> list = new List<T>();
            foreach (var item in Ids)
            {
                T? searched = GetById(item, out returnCode, out returnMessage);
                if (searched != null)
                {
                    list.Add(searched);
                }
            }
            return list;
        }
        /// <summary>
        /// Function to insert childData in sub table.
        /// </summary>
        /// <param name="newData"></param>
        /// <param name="oldData"></param>
        /// <param name="transaction"></param>
        /// <returns></returns>
        public virtual int InsertChildData(T data, IDbTransaction? transaction)
        {
            return 1;
        }
        /// <summary>
        /// Function to get childData in sub table.
        /// </summary>
        /// <param name="newData"></param>
        /// <param name="oldData"></param>
        /// <param name="connection"></param>
        /// <returns></returns>
        public virtual int GetChildData(T data, IDbConnection? connection)
        {
            return 1;
        }
        /// <summary>
        /// Function to update childData in sub table. Should process follow thread delete all -> reinsert all.
        /// </summary>
        /// <param name="newData"></param>
        /// <param name="oldData"></param>
        /// <param name="transaction"></param>
        /// <returns></returns>
        public virtual int UpdateChildData(T newData, T oldData, IDbTransaction? transaction)
        {
            DeleteChildData(oldData, transaction);
            InsertChildData(newData, transaction);
            return 1;
        }

        public virtual int DeleteChildData(T data, IDbTransaction? transaction)
        {
            return 1;
        }


        public virtual bool BeforeInsert(T entity, ref List<int> returnCode, ref List<string> returnMessage)
        {
            return true;
        }

        public virtual bool BeforeUpdate(T entity, T oldData, ref List<int> returnCode, ref List<string> returnMessage)
        {
            return true;
        }

        public virtual bool BeforeDelete(T entity, ref List<int> returnCode, ref List<string> returnMessage)
        {
            return true;
        }
    }
}
