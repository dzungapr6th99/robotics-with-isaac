using System.Data;
using BusinessLayer.Interfaces;
using DataAccess.Interface;
using DataAccess.Interfaces;
using DbObject;
using DbObject.Common;

namespace BusinessLayer
{
    public class RobotTaskTemplateBL : BaseBL<RobotTaskTemplate>, IRobotTaskTemplateBL
    {
        public override string TableName => DatabaseEnum.TableName.RobotTaskTemplate;
        private readonly IBaseDA<Map> _mapBaseDA;
        private readonly IBaseDA<Point> _pointBaseDA;

        public RobotTaskTemplateBL(IBaseDA<RobotTaskTemplate> baseDA, IBaseDA<Map> mapBaseDA, IBaseDA<Point> pointBaseDA, IDbManagement dbManagement) : base(baseDA, dbManagement)
        {
            _mapBaseDA = mapBaseDA;
            _pointBaseDA = pointBaseDA;
        }

        public override int GetChildData(RobotTaskTemplate data, IDbConnection? connection)
        {
            if (connection != null)
            {
                data.FromPoint = _pointBaseDA.Query(new Point()
                {
                   Id = data.FromPointId,
                   MapId = data.MapId
                }, connection)?.FirstOrDefault();
                data.FromPoint = _pointBaseDA.Query(new Point()
                {
                    Id = data.ToPointId,
                    MapId = data.MapId
                }, connection)?.FirstOrDefault();
            }
            return base.GetChildData(data, connection);
        }


    }
}
