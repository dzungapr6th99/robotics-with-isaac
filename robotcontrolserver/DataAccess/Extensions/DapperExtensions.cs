using CommonLib;
using Dapper;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Data;
using System.Linq;
using System.Reflection;
using System.Threading.Tasks;
using static Dapper.SqlMapper;
using DbObject.Attributes;
using NLog;
namespace DataAccess.Extensions
{
    public static class DapperExtensions
    {
        #region Count

        public static int Count<T>(this IDbConnection connection, object param = null, IDbTransaction transaction = null, int? commandTimeout = null) where T : class
        {
            string sqlText = typeof(T).BuildCountSqlText(param);
            return connection.QueryFirstOrDefault<int>(sqlText, param, transaction, commandTimeout: commandTimeout, commandType: CommandType.Text);
        }

        public static async Task<int> CountAsync<T>(this IDbConnection connection, object param = null, IDbTransaction transaction = null, int? commandTimeout = null) where T : class
        {
            string sqlText = typeof(T).BuildCountSqlText(param);
            return await connection.QueryFirstOrDefaultAsync<int>(sqlText, param, transaction, commandTimeout: commandTimeout, commandType: CommandType.Text);
        }

        #endregion

        #region Get

        /// <summary>
        /// DUNGNT Bổ sung hàm Get truyền vào type và param
        /// </summary>
        /// <param name="connection"></param>
        /// <param name="param"></param>
        /// <param name="transaction"></param>
        /// <param name="commandTimeout"></param>
        /// <returns></returns>
        public static IEnumerable Get(this IDbConnection connection, Type type, object param = null, IDbTransaction transaction = null, int? commandTimeout = null)
        {
            string sqlText = type.BuildGetSqlText(param);
            return connection.Query(type, sqlText, param, transaction, commandTimeout: commandTimeout, commandType: CommandType.Text);
        }

        public static IEnumerable<T> Get<T>(this IDbConnection connection, object? param = null, IDbTransaction transaction = null, int? commandTimeout = null) where T : class
        {
            string sqlText = typeof(T).BuildGetSqlText(param);
            return connection.Query<T>(sqlText, param, transaction, commandTimeout: commandTimeout, commandType: CommandType.Text);
        }

        public static async Task<IEnumerable<T>> GetAsync<T>(this IDbConnection connection, object param = null, IDbTransaction transaction = null, int? commandTimeout = null) where T : class
        {
            string sqlText = typeof(T).BuildGetSqlText(param);
            return await connection.QueryAsync<T>(sqlText, param, transaction, commandTimeout: commandTimeout, commandType: CommandType.Text);
        }

        public static IEnumerable<T> GetView<T>(this IDbConnection connection, object param = null, IDbTransaction transaction = null, int? commandTimeout = null) where T : class
        {
            string sqlText = typeof(T).BuildGetViewSqlText(param);
            return connection.Query<T>(sqlText, param, transaction, commandTimeout: commandTimeout, commandType: CommandType.Text);
        }

        public static async Task<IEnumerable<T>> GetViewAsync<T>(this IDbConnection connection, object param = null, IDbTransaction transaction = null, int? commandTimeout = null) where T : class
        {
            string sqlText = typeof(T).BuildGetViewSqlText(param);
            return await connection.QueryAsync<T>(sqlText, param, transaction, commandTimeout: commandTimeout, commandType: CommandType.Text);
        }
        ///// <summary>
        ///// Bổ sung lấy bản ghi từ database
        ///// </summary>
        ///// <typeparam name="T"></typeparam>
        ///// <param name="connection"></param>
        ///// <param name="param"></param>
        ///// <param name="transaction"></param>
        ///// <param name="commandTimeout"></param>
        ///// <returns></returns>
        //public static T GetFirstOrDefault<T>(this IDbConnection connection, object param = null, IDbTransaction transaction = null, int? commandTimeout = null) where T : class
        //{
        //    var sqlText = typeof(T).BuildGetSqlText(param);
        //    return connection.QueryFirstOrDefault<T>(sqlText, param, transaction, commandTimeout: commandTimeout, commandType: CommandType.Text);
        //}
        public static T GetFirstOrDefault<T>(this IDbConnection connection, object param = null, IDbTransaction transaction = null, int? commandTimeout = null) where T : class
        {
            var sqlText = typeof(T).BuildGetSqlText(param);
            return connection.QueryFirstOrDefault<T>(sqlText, param, transaction, commandTimeout: commandTimeout, commandType: CommandType.Text);
        }

        public static async Task<T> GetFirstOrDefaultAsync<T>(this IDbConnection connection, object param = null, IDbTransaction transaction = null, int? commandTimeout = null) where T : class
        {
            var sqlText = typeof(T).BuildGetSqlText(param);
            return await connection.QueryFirstOrDefaultAsync<T>(sqlText, param, transaction, commandTimeout: commandTimeout, commandType: CommandType.Text);
        }

        public static T GetViewFirstOrDefault<T>(this IDbConnection connection, object param = null, IDbTransaction transaction = null, int? commandTimeout = null) where T : class
        {
            var sqlText = typeof(T).BuildGetViewSqlText(param);
            return connection.QueryFirstOrDefault<T>(sqlText, param, transaction, commandTimeout: commandTimeout, commandType: CommandType.Text);
        }

        public static async Task<T> GetViewFirstOrDefaultAsync<T>(this IDbConnection connection, object param = null, IDbTransaction transaction = null, int? commandTimeout = null) where T : class
        {
            var sqlText = typeof(T).BuildGetViewSqlText(param);
            return await connection.QueryFirstOrDefaultAsync<T>(sqlText, param, transaction, commandTimeout: commandTimeout, commandType: CommandType.Text);
        }

        #endregion

        #region Insert
        /// <summary>
        /// DUNGNT Bổ sung hàm này truyển thẳng Type insert không sử dụng generic
        /// </summary>
        /// <param name="connection"></param>
        /// <param name="entity"></param>
        /// <param name="type"></param>
        /// <param name="transaction"></param>
        /// <param name="commandTimeout"></param>
        /// <returns></returns>
        public static int Insert(this IDbConnection connection, object entity, Type type, IDbTransaction transaction = null, int? commandTimeout = null)
        {
            var insertSqlText = type.BuildInsertSqlText();
            return transaction.Connection.Execute(insertSqlText, entity, transaction, commandTimeout, CommandType.Text);
        }

        public static int Insert<T>(this IDbConnection connection, T entity, IDbTransaction transaction = null, int? commandTimeout = null)
        {
            var type = GetEntityType(entity);
            var insertSqlText = type.BuildInsertSqlText();
            return transaction.Connection.Execute(insertSqlText, entity, transaction, commandTimeout, CommandType.Text);
        }

        public static async Task<int> InsertAsync<T>(this IDbConnection connection, T entity, IDbTransaction transaction = null, int? commandTimeout = null)
        {
            var type = GetEntityType(entity);
            var insertSqlText = type.BuildInsertSqlText();
            return await transaction.Connection.ExecuteAsync(insertSqlText, entity, transaction, commandTimeout, CommandType.Text);
        }

        public static int InsertWithClob<T>(this IDbConnection connection, T entity, IDbTransaction transaction = null, int? commandTimeout = null)
        {
            return InsertWithClobAsync<T>(connection, entity, transaction, commandTimeout).GetAwaiter().GetResult();
        }

        public static async Task<int> InsertWithClobAsync<T>(this IDbConnection connection, T entity, IDbTransaction transaction = null, int? commandTimeout = null)
        {
            var type = GetEntityType(entity);
            var insertSqlText = type.BuildCreateSqlTextWithClob(entity, out var executeParams);
            return await connection.ExecuteAsync(insertSqlText, executeParams, transaction, commandTimeout, CommandType.Text);
        }

        #endregion

        #region Update
        /// <summary>
        /// Cập nhật thông tin truyền thẳng type và object to change - DUNGNT
        /// </summary>
        /// <typeparam name="T"></typeparam>
        /// <param name="connection"></param>
        /// <param name="entity"></param>
        /// <param name="param"></param>
        /// <param name="transaction"></param>
        /// <param name="commandTimeout"></param>
        /// <returns></returns>
        public static int Update(this IDbConnection connection, object entity, object param, Type type, IDbTransaction transaction = null, int? commandTimeout = null)
        {
            var updateSqlText = type.BuildUpdateSqlText(entity, param, out var executeParams);
            return transaction.Connection.Execute(updateSqlText, executeParams, transaction, commandTimeout, CommandType.Text);
        }

        public static async Task<int> UpdateAsync(this IDbConnection connection, object entity, object param, Type type, IDbTransaction transaction = null, int? commandTimeout = null)
        {
            var updateSqlText = type.BuildUpdateSqlText(entity, param, out var executeParams);
            return await transaction.Connection.ExecuteAsync(updateSqlText, executeParams, transaction, commandTimeout, CommandType.Text);
        }

        public static int Update<T, H>(this IDbConnection connection, T entity, H param, IDbTransaction transaction = null, int? commandTimeout = null)
        {
            var type = GetEntityType(entity);
            var updateSqlText = type.BuildUpdateSqlText(entity, param, out var executeParams);
            return transaction.Connection.Execute(updateSqlText, executeParams, transaction, commandTimeout, CommandType.Text);
        }

        public static async Task<int> UpdateAsync<T, H>(this IDbConnection connection, T entity, H param, IDbTransaction transaction = null, int? commandTimeout = null)
        {
            var type = GetEntityType(entity);
            var updateSqlText = type.BuildUpdateSqlText(entity, param, out var executeParams);
            return await transaction.Connection.ExecuteAsync(updateSqlText, executeParams, transaction, commandTimeout, CommandType.Text);
        }

        #endregion

        #region Delete

        /// <summary>
        /// DUNGNT Thêm delete truyền vào type và object
        /// </summary>
        /// <param name="connection"></param>
        /// <param name="entity"></param>
        /// <param name="type"></param>
        /// <param name="transaction"></param>
        /// <param name="commandTimeout"></param>
        /// <returns></returns>
        public static int Delete(this IDbConnection connection, object entity, Type type, IDbTransaction transaction = null, int? commandTimeout = null)
        {
            var deleteSqlText = type.BuildDeleteSqlText();
            return transaction.Connection.Execute(deleteSqlText, entity, transaction, commandTimeout, CommandType.Text);
        }

        public static int Delete<T>(this IDbConnection connection, T entity, IDbTransaction transaction = null, int? commandTimeout = null)
        {
            var type = GetEntityType(entity);
            var deleteSqlText = type.BuildDeleteSqlText();
            return transaction.Connection.Execute(deleteSqlText, entity, transaction, commandTimeout, CommandType.Text);
        }

        public static async Task<int> DeleteAsync<T>(this IDbConnection connection, T entity, IDbTransaction transaction = null, int? commandTimeout = null)
        {
            var type = GetEntityType(entity);
            var deleteSqlText = type.BuildDeleteSqlText();
            return await transaction.Connection.ExecuteAsync(deleteSqlText, entity, transaction, commandTimeout, CommandType.Text);
        }

        /// <summary>
        /// Delete dữ liệu T với điều kiện truyền vào khác T
        /// </summary>
        /// <typeparam name="T"></typeparam>
        /// <param name="connection"></param>
        /// <param name="param"></param>
        /// <param name="transaction"></param>
        /// <param name="commandTimeout"></param>
        /// <returns></returns>
        public static int Delete<T>(this IDbConnection connection, object param, IDbTransaction transaction = null, int? commandTimeout = null) where T : class, new()
        {
            return DeleteAsync<T>(connection, param, transaction, commandTimeout).GetAwaiter().GetResult();
        }

        /// <summary>
        /// Delete dữ liệu T với điều kiện truyền vào khác T
        /// </summary>
        /// <typeparam name="T"></typeparam>
        /// <param name="connection"></param>
        /// <param name="param"></param>
        /// <param name="transaction"></param>
        /// <param name="commandTimeout"></param>
        /// <returns></returns>
        public static async Task<int> DeleteAsync<T>(this IDbConnection connection, object param, IDbTransaction transaction = null, int? commandTimeout = null) where T : class, new()
        {
            var type = typeof(T);
            var deleteSqlText = type.BuildDeleteSqlText(param);
            return await transaction.Connection.ExecuteAsync(deleteSqlText, param, transaction, commandTimeout, CommandType.Text);
        }

        #endregion


        #region Private functions 

        private static string BuildCountSqlText(this Type type, object param)
        {
            var tableName = type.GetDbTableName();

            string sqlText = $@"SELECT COUNT(*) FROM {tableName}";

            if (param != null)
            {
                var condKeys = new List<string>();
                if (param.IsDictionary())
                {
                    var dicParam = (IDictionary<string, object>)param;
                    foreach (var key in dicParam.Keys)
                    {
                        var paramValue = dicParam[key];
                        if (paramValue.GetType().IsArray || paramValue.IsEnumerable() || paramValue.IsList())
                        {
                            condKeys.Add($"{key} IN :{key}");
                        }
                        else
                        {
                            condKeys.Add($"{key} = :{key}");
                        }
                    }
                }
                else
                {
                    var props = param.GetType().GetProperties();
                    foreach (var prop in props)
                    {
                        var propValue = GetPropertyValue(param, prop.Name);
                        if (propValue.GetType().IsArray || propValue.IsEnumerable() || propValue.IsList())
                        {
                            condKeys.Add($"{prop.Name} IN :{prop.Name}");
                        }
                        else
                        {
                            condKeys.Add($"{prop.Name} = :{prop.Name}");
                        }
                    }
                }

                if (condKeys.Count > 0)
                {
                    sqlText += $@" WHERE {string.Join(" AND ", condKeys)}";
                }
            }

            CommonLog.logDb.Info($@"SQL Text: {sqlText}");
            return sqlText;
        }

        private static string BuildGetSqlText(this Type type, object? param)
        {
            var tableName = type.GetDbTableName().ToUpper();

            string sqlText = $@"SELECT * FROM {tableName}";

            //if (param != null)
            //{
            //    var condKeys = new List<string>();
            //    if (param.IsDictionary())
            //    {
            //        condKeys.AddRange(((IDictionary<string, object>)param).Keys);
            //    }
            //    else
            //    {
            //        condKeys.AddRange(param.GetType().GetProperties().Select(x => x.Name));
            //    }

            //    if (condKeys.Count > 0)
            //    {
            //        sqlText += $@" WHERE {string.Join(" AND ", condKeys.Select(key => $" {key} = :{key} "))}";
            //    }
            //}

            if (param != null)
            {
                var condKeys = new List<string>();
                if (param.IsDictionary())
                {
                    var dicParam = (IDictionary<string, object>)param;
                    foreach (var key in dicParam.Keys)
                    {
                        var paramValue = dicParam[key];
                        if (paramValue.GetType().IsArray || paramValue.IsEnumerable() || paramValue.IsList())
                        {
                            condKeys.Add($"{key.ToUpper()} IN :{key}");
                        }
                        else
                        {
                            condKeys.Add($"{key.ToUpper()} = :{key}");
                        }
                    }
                }
                else
                {
                    var props = param.GetType().GetProperties();
                    foreach (var prop in props)
                    {
                        var propValue = GetPropertyValue(param, prop.Name);
                        var attribute = (DbFieldAttribute)prop.GetCustomAttributes(typeof(DbFieldAttribute))?.FirstOrDefault();
                        if (propValue == null || (attribute != null && attribute.Ignore))
                        {
                            continue;
                        }
                        else if (propValue.GetType().IsArray || propValue.IsEnumerable() || propValue.IsList())
                        {
                            condKeys.Add($"{prop.Name.ToUpper()} IN :{prop.Name}");
                        }
                        else
                        {
                            condKeys.Add($"{prop.Name.ToUpper()} = :{prop.Name}");
                        }
                    }
                }

                if (condKeys.Count > 0)
                {
                    sqlText += $@" WHERE {string.Join(" AND ", condKeys)}";
                }
            }

            CommonLog.logDb.Info($@"SQL Text: {sqlText}");
            return sqlText;
        }

        private static string BuildGetViewSqlText<T>(this Type type, T param)
        {
            var viewName = type.GetDbViewName();

            string sqlText = $@"SELECT * FROM {viewName}";

            //if (param != null)
            //{
            //    var condKeys = new List<string>();
            //    if (param.IsDictionary())
            //    {
            //        condKeys.AddRange(((IDictionary<string, object>)param).Keys);
            //    }
            //    else
            //    {
            //        condKeys.AddRange(param.GetType().GetProperties().Select(x => x.Name));
            //    }

            //    if (condKeys.Count > 0)
            //    {
            //        sqlText += $@" WHERE {string.Join(" AND ", condKeys.Select(key => $" {key} = :{key} "))}";
            //    }
            //}

            if (param != null)
            {
                var condKeys = new List<string>();
                if (param.IsDictionary())
                {
                    var dicParam = (IDictionary<string, object>)param;
                    foreach (var key in dicParam.Keys)
                    {
                        var paramValue = dicParam[key];
                        if (paramValue.GetType().IsArray || paramValue.IsEnumerable() || paramValue.IsList())
                        {
                            condKeys.Add($"{key} IN :{key}");
                        }
                        else
                        {
                            condKeys.Add($"{key} = :{key}");
                        }
                    }
                }
                else
                {
                    var props = param.GetType().GetProperties();
                    foreach (var prop in props)
                    {
                        var propValue = GetPropertyValue(param, prop.Name);
                        if (propValue.GetType().IsArray || propValue.IsEnumerable() || propValue.IsList())
                        {
                            condKeys.Add($"{prop.Name} IN :{prop.Name}");
                        }
                        else
                        {
                            condKeys.Add($"{prop.Name} = :{prop.Name}");
                        }
                    }
                }

                if (condKeys.Count > 0)
                {
                    sqlText += $@" WHERE {string.Join(" AND ", condKeys)}";
                }
            }

            CommonLog.logDb.Info($@"SQL Text: {sqlText}");
            return sqlText;
        }

        private static string BuildInsertSqlText(this Type type)
        {
            var tableName = type.GetDbTableName().ToUpper();
            var insertFieldNames = new List<string>();
            var insertFieldProps = new List<string>();

            var props = type.GetProperties();
            foreach (var prop in props)
            {
                var attribute = (DbFieldAttribute)prop.GetCustomAttributes(typeof(DbFieldAttribute))?.FirstOrDefault();
                if (attribute != null && attribute.IgnoreInsert)
                {
                    continue;
                }
                // DUNGNT Bổ sung thêm TH nếu là Detail Table thì bỏ qua không insert
                // Sẽ chạy vào Insert child sau khi insert
                if (attribute != null && attribute.IsDetailTable)
                {
                    continue;
                }
                var fieldName = !string.IsNullOrEmpty(attribute?.Name) ? attribute.Name : prop.Name;
                insertFieldNames.Add(fieldName.ToUpper());
                insertFieldProps.Add(prop.Name);
            }

            String sqlText = $@"INSERT INTO {tableName.ToUpper()}({string.Join(", ", insertFieldNames)}) VALUES ({string.Join(", ", insertFieldProps.Select(field => ":" + field))})";
            CommonLog.logDb.Info($@"SQL Text: {sqlText}");
            return sqlText;
        }

        private static string BuildCreateSqlTextWithClob<T>(this Type type, T entity, out DynamicParameters executeParams)
        {
            var tableName = type.GetDbTableName();
            var insertFieldNames = new List<string>();
            var insertFieldProps = new List<string>();

            executeParams = new();

            var props = type.GetProperties();
            foreach (var prop in props)
            {
                var attribute = (DbFieldAttribute)prop.GetCustomAttributes(typeof(DbFieldAttribute))?.FirstOrDefault();
                if (attribute != null && attribute.IgnoreInsert)
                {
                    continue;
                }

                // DUNGNT Bổ sung thêm TH nếu là Detail Table thì bỏ qua không insert
                // Sẽ chạy vào Insert child sau khi insert
                if (attribute != null && attribute.IsDetailTable)
                {
                    continue;
                }

                //
                var executeValue = entity.GetPropertyValue(prop.Name);
                var executeValueAsString = executeValue as string;

                var fieldName = !string.IsNullOrEmpty(attribute?.Name) ? attribute.Name : prop.Name;
                insertFieldNames.Add(fieldName);

                //
                int clobChunkSize = 3000;
                if (attribute?.IsClob == true && !string.IsNullOrEmpty(executeValueAsString) && executeValueAsString.Length > clobChunkSize)
                {
                    var chunkedStrings = executeValueAsString.Chunk(clobChunkSize).Select(x => new string(x)).ToList();
                    var clobJoinFields = new List<string>();
                    for (int i = 0; i < chunkedStrings.Count; i++)
                    {
                        var chunkedString = chunkedStrings[i];
                        clobJoinFields.Add($"TO_CLOB(:{prop.Name}{i})");
                        executeParams.Add($"{prop.Name}{i}", chunkedString);
                    }
                    insertFieldProps.Add(string.Join("||", clobJoinFields));
                }
                else
                {
                    insertFieldProps.Add($":{prop.Name}");
                    executeParams.Add(prop.Name, executeValue);
                }
            }

            String sqlText = $@"INSERT INTO {tableName}({string.Join(", ", insertFieldNames)}) VALUES ({string.Join(", ", insertFieldProps.Select(field => field))})";
            CommonLog.logDb.Info($@"SQL Text: {sqlText}");
            return sqlText;
        }

        private static string BuildUpdateSqlText<T, H>(this Type type, T entity, H param, out DynamicParameters executeParams)
        {
            var tableName = type.GetDbTableName();
            var updateFieldNames = new List<string>();
            var condFieldNames = new List<string>();
            executeParams = new();

            // Xu ly dieu kien neu co truyen vao
            var hasSetupParams = false;
            if (param != null)
            {
                if (param.IsDictionary())
                {
                    var dicParam = (IDictionary<string, object>)param;
                    if (dicParam.Keys.Count > 0)
                    {
                        hasSetupParams = true;

                        foreach (var key in dicParam.Keys)
                        {
                            string newParamName = $"p_{key}";
                            var newParamValue = dicParam[key];
                            if (newParamValue.GetType().IsArray || newParamValue.IsEnumerable() || newParamValue.IsList())
                            {
                                condFieldNames.Add($"{key.ToUpper()} IN :{newParamName}");
                            }
                            else
                            {
                                condFieldNames.Add($"{key.ToUpper()} = :{newParamName}");
                            }
                            executeParams.Add(newParamName, newParamValue);
                        }
                    }
                }
                else
                {
                    var condKeys = typeof(T).Equals(typeof(H)) ? param.GetKeys() : param.GetType().GetProperties().Select(x => x.Name).ToList();
                    if (condKeys.Count > 0)
                    {
                        hasSetupParams = true;

                        foreach (var key in condKeys)
                        {
                            string newParamName = $"p_{key}";
                            var newParamValue = param.GetPropertyValue(key);

                            if (newParamValue.GetType().IsArray || newParamValue.IsEnumerable() || newParamValue.IsList())
                            {
                                condFieldNames.Add($"{key.ToUpper()} IN :{newParamName}");
                            }
                            else
                            {
                                condFieldNames.Add($"{key.ToUpper()} = :{newParamName}");
                            }
                            executeParams.Add(newParamName, newParamValue);
                        }
                    }
                }
            }

            //
            if (typeof(T).Equals(type))
            {
                var props = type.GetProperties();
                foreach (var prop in props)
                {
                    bool needAddToExecuteParams = false;

                    var attribute = (DbFieldAttribute)prop.GetCustomAttributes(typeof(DbFieldAttribute))?.FirstOrDefault();
                    var fieldName = !string.IsNullOrEmpty(attribute?.Name) ? attribute.Name : prop.Name;

                    // Check truong thong tin co duoc update khong de build bien trong Update
                    if (attribute == null || (!attribute.IgnoreUpdate && !attribute.IsKey))
                    {
                        updateFieldNames.Add($"{fieldName.ToUpper()} = :{prop.Name}");
                        needAddToExecuteParams = true;
                    }

                    // Check truong thong tin co la Key ban ghi khong de buid Dieu kien
                    if (!hasSetupParams && attribute != null && attribute.IsKey)
                    {
                        condFieldNames.Add($"{fieldName.ToUpper()} = :{prop.Name}");
                        needAddToExecuteParams = true;
                    }

                    if (needAddToExecuteParams)
                    {
                        string value = prop.GetValue(entity).ToString();
                        executeParams.Add(prop.Name.ToUpper(), value: prop.GetValue(entity));
                    }
                }
            }
            else
            {
                if (entity.IsDictionary())
                {
                    var dicEntity = (IDictionary<string, object>)entity;
                    foreach (var key in dicEntity.Keys)
                    {
                        updateFieldNames.Add($"{key.ToUpper()} = :{key}");
                        executeParams.Add(key.ToUpper(), value: dicEntity[key]);
                    }
                }
                else
                {
                    var props = entity.GetType().GetProperties();
                    foreach (var prop in props)
                    {
                        updateFieldNames.Add($"{prop.Name.ToUpper()} = :{prop.Name}");
                        executeParams.Add(prop.Name.ToUpper(), value: prop.GetValue(entity));
                    }
                }
            }

            String sqlText = $"UPDATE {tableName} SET {string.Join(", ", updateFieldNames)} WHERE {string.Join(" AND ", condFieldNames)}";
            CommonLog.logDb.Info($@"SQL Text: {sqlText}");
            return sqlText;
        }

        private static string BuildDeleteSqlText(this Type type, object param = null)
        {
            var tableName = type.GetDbTableName();
            var condFieldNames = new List<string>();

            if (GetEntityType(param).Equals(type) || param is null)
            {
                var props = type.GetProperties();
                foreach (var prop in props)
                {
                    var attribute = (DbFieldAttribute)prop.GetCustomAttributes(typeof(DbFieldAttribute))?.FirstOrDefault();
                    if (attribute == null || !attribute.IsKey)
                    {
                        continue;
                    }

                    var fieldName = !string.IsNullOrEmpty(attribute.Name) ? attribute.Name : prop.Name;
                    condFieldNames.Add($"{fieldName} = :{prop.Name}");
                }
            }
            else if (param?.IsDictionary() == true)
            {
                var dicParam = (IDictionary<string, object>)param;
                foreach (var key in dicParam.Keys)
                {
                    var keyValue = dicParam[key];
                    if (keyValue.GetType().IsArray || keyValue.IsEnumerable() || keyValue.IsList())
                    {
                        condFieldNames.Add($"{key.ToUpper()} IN :{key}");
                    }
                    else
                    {
                        condFieldNames.Add($"{key.ToUpper()} = :{key}");
                    }
                }
            }
            else
            {
                var props = param.GetType().GetProperties();
                foreach (var prop in props)
                {
                    var propValue = prop.GetValue(param);
                    if (propValue.GetType().IsArray || propValue.IsEnumerable() || propValue.IsList())
                    {
                        condFieldNames.Add($"{prop.Name.ToUpper()} IN :{prop.Name}");
                    }
                    else
                    {
                        condFieldNames.Add($"{prop.Name.ToUpper()} = :{prop.Name}");
                    }
                }
            }

            String sqlText = $@"DELETE FROM {tableName} WHERE {string.Join(" AND ", condFieldNames)}";
            CommonLog.logDb.Info($@"SQL Text: {sqlText}");
            return sqlText;
        }

        private static Type GetEntityType<T>(T entity)
        {
            var type = typeof(T);

            if (type.IsArray)
            {
                type = type.GetElementType();
            }
            else if (type.IsGenericType)
            {
                var typeInfo = type.GetTypeInfo();
                bool implementsGenericIEnumerableOrIsGenericIEnumerable =
                    typeInfo.ImplementedInterfaces.Any(ti => ti.IsGenericType && ti.GetGenericTypeDefinition() == typeof(IEnumerable<>)) ||
                    typeInfo.GetGenericTypeDefinition() == typeof(IEnumerable<>);

                if (implementsGenericIEnumerableOrIsGenericIEnumerable)
                {
                    type = type.GetGenericArguments()[0];
                }
            }

            return type;
        }

        private static string GetDbTableName(this Type type)
        {
            var dbTableAttribute = (DbTableAttribute)type.GetCustomAttributes(typeof(DbTableAttribute))?.FirstOrDefault();
            if (dbTableAttribute != null && !string.IsNullOrEmpty(dbTableAttribute.Name))
            {
                return dbTableAttribute.Name;
            }

            return type.Name;
        }

        private static string GetDbViewName(this Type type)
        {
            var dbTableAttribute = (DbTableAttribute)type.GetCustomAttributes(typeof(DbTableAttribute))?.FirstOrDefault();
            if (dbTableAttribute != null && !string.IsNullOrEmpty(dbTableAttribute.ViewName))
            {
                return dbTableAttribute.ViewName;
            }

            return type.Name;
        }

        private static List<string> GetKeys<T>(this T entity)
        {
            var keys = new List<string>();

            var paramProps = entity.GetType().GetProperties();
            foreach (var prop in paramProps)
            {
                var attribute = (DbFieldAttribute)prop.GetCustomAttributes(typeof(DbFieldAttribute))?.FirstOrDefault();
                if (attribute == null || !attribute.IsKey)
                {
                    continue;
                }

                var fieldName = !string.IsNullOrEmpty(attribute.Name) ? attribute.Name : prop.Name;
                keys.Add(fieldName);
            }

            return keys;
        }

        private static object GetPropertyValue(this object obj, string propertyName)
        {
            if (obj == null || string.IsNullOrEmpty(propertyName)) return null;
            PropertyInfo propertyInfo = obj.GetType().GetProperty(propertyName, BindingFlags.IgnoreCase | BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance);
            if (propertyInfo == null || !propertyInfo.CanRead) return null;
            return propertyInfo.GetValue(obj);
        }

        private static bool IsList(this object o)
        {
            if (o == null) return false;
            return o is IList &&
                   o.GetType().IsGenericType &&
                   o.GetType().GetGenericTypeDefinition().IsAssignableFrom(typeof(List<>));
        }

        private static bool IsEnumerable(this object o)
        {
            //return o.GetType() is IEnumerable;
            return typeof(IEnumerable).IsAssignableFrom(o.GetType()) && !o.GetType().Equals(typeof(string));
        }

        private static bool IsDictionary(this object o)
        {
            if (o == null) return false;
            return o is IDictionary &&
                   o.GetType().IsGenericType &&
                   o.GetType().GetGenericTypeDefinition().IsAssignableFrom(typeof(Dictionary<,>));
        }

        #endregion
    }
}
