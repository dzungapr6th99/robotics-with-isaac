using DataAccess.Interfaces;
using DbObject;
using DataAccess.Extensions;
using System.Data;
namespace DataAccess
{
    public class MapDA : BaseDA<Map>, IMapDA
    {
        public MapDA(): base()
        {

        }


        public override int Insert(Map obj, IDbTransaction transaction)
        {
            // Gọi base method để thực hiện insert
            return base.Insert(obj, transaction);
        }

        public override int Update(Map newData, Map oldData, IDbTransaction transaction)
        {
            // Gọi base method để thực hiện update
            return base.Update(newData, oldData, transaction);
        }

        public override int Delete(Map obj, IDbTransaction transaction)
        {
            // Có thể thêm logic kiểm tra trước khi xóa
            return base.Delete(obj, transaction);
        }

        public override List<Map> Query(object? param, IDbConnection connection)
        {
            if (connection != null)
            {
                return connection.Get<Map>(param).ToList();
            }
            return new List<Map>();
        }

    }
}
