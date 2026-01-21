using DataAccess.Interfaces;
using System.Data.SQLite;
using System.Data;
using DataAccess.Interface;

namespace DataAccess.Helper
{
    public class DbManagement : IDbManagement
    {
        private readonly string _connectionString;
        public DbManagement(string connectionString)
        {
            _connectionString = connectionString;
        }
        public IDbConnection GetConnection()
        {
            SQLiteConnection connection = new SQLiteConnection(_connectionString);
            connection.Open();
            return connection;
        }

    }
}
