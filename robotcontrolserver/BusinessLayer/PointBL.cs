using BusinessLayer.Interfaces;
using CommonLib;
using DataAccess.Interface;
using DataAccess.Interfaces;
using DbObject;
using DbObject.Common;
using System.Data;

namespace BusinessLayer
{
    public class PointBL : BaseBL<Point>, IPointBL
    {
        private readonly IBaseDA<PointType> _pointTypeDA;
        private readonly IBaseDA<Map> _mapDA;
        public override string TableName => DatabaseEnum.TableName.Point;
        public PointBL(IBaseDA<Point> baseDA, IBaseDA<PointType> pointTypeDA, IBaseDA<Map> mapDA,IDbManagement dbManagement) : base(baseDA, dbManagement)
        {
            _pointTypeDA = pointTypeDA;
            _mapDA = mapDA;
        }

        public override int GetChildData(Point data, IDbConnection? connection)
        {
            if (connection != null)
            {
                data.PointType = _pointTypeDA.Query(new PointType { Id = data.PointTypeId }, connection)?.FirstOrDefault();
            }

            return base.GetChildData(data, connection);
        }

        public override bool BeforeInsert(Point entity, IDbConnection connection, ref List<int> returnCode, ref List<string> returnMessage)
        {
            Map? map = _mapDA.Query(new Map()
            {
                Id = entity.MapId
            }, connection)?.FirstOrDefault();
            if ( map == null)
            {
                returnCode.Add(ConstData.ReturnCode.Point.MAP_NOT_EXIST_IN_DATABASE);
                returnMessage.Add(ConstData.ReturnMessage.Point.MAP_NOT_EXIST_IN_DATABASE);
                return false;
            }
            return true;
        }

        public override bool BeforeUpdate(Point entity, Point oldData, IDbConnection connection, ref List<int> returnCode, ref List<string> returnMessage)
        {
            if (entity.MapId != oldData.MapId)
            {
                returnCode.Add(ConstData.ReturnCode.Point.MAP_NOT_EXIST_IN_DATABASE);
                returnMessage.Add(ConstData.ReturnMessage.Point.MAP_NOT_EXIST_IN_DATABASE);
                return false;
            }    
            return true;
        }

        public bool AlignPoints(List<Point> points, out int returnCode, out string returnMessage, out List<string> details)
        {
            details = new List<string>();
            returnCode = ConstData.ReturnCode.SUCCESS;
            returnMessage = ConstData.ReturnMessage.SUCCESS;

            if (points == null || points.Count < 2)
            {
                details.Add("At least two points are required to align.");
                returnCode = ConstData.ReturnCode.SERVICE_GET_ERROR;
                returnMessage = ConstData.ReturnMessage.SERVICE_GET_ERROR;
                return false;
            }

            try
            {
                using (var connection = _dbManagement.GetConnection())
                using (var transaction = connection.BeginTransaction())
                {
                    var firstId = points.First().Id;
                    var lastId = points.Last().Id;
                    if (!firstId.HasValue || !lastId.HasValue)
                    {
                        details.Add("First and last point must have Id.");
                        return false;
                    }
                    var firstPoint = _baseDA.Query(new Point { Id = firstId }, connection)?.FirstOrDefault();
                    var lastPoint = _baseDA.Query(new Point { Id = lastId }, connection)?.FirstOrDefault();
                    if (firstPoint == null || lastPoint == null)
                    {
                        details.Add("Cannot find first or last point in database.");
                        return false;
                    }
                    if (!firstPoint.X.HasValue || !firstPoint.Y.HasValue || !lastPoint.X.HasValue || !lastPoint.Y.HasValue)
                    {
                        details.Add("First and last point must have coordinates.");
                        return false;
                    }

                    double dx = (lastPoint.X.Value - firstPoint.X.Value) / (points.Count - 1);
                    double dy = (lastPoint.Y.Value - firstPoint.Y.Value) / (points.Count - 1);

                    for (int i = 0; i < points.Count; i++)
                    {
                        var p = points[i];
                        if (!p.Id.HasValue)
                        {
                            details.Add($"Point at index {i} missing Id.");
                            continue;
                        }
                        var dbPoint = _baseDA.Query(new Point { Id = p.Id }, connection)?.FirstOrDefault();
                        if (dbPoint == null)
                        {
                            details.Add($"Point id {p.Id} not found.");
                            continue;
                        }
                        Point updated = new Point
                        {
                            Id = dbPoint.Id,
                            MapId = dbPoint.MapId,
                            Name = dbPoint.Name,
                            PointTypeId = dbPoint.PointTypeId,
                            X = firstPoint.X + dx * i,
                            Y = firstPoint.Y + dy * i
                        };
                        _baseDA.Update(updated, dbPoint, transaction);
                    }

                    transaction.Commit();
                    return details.Count == 0;
                }
            }
            catch (Exception ex)
            {
                CommonLog.logDb.Error(ex);
                returnCode = ConstData.ReturnCode.SERVICE_GET_ERROR;
                returnMessage = ConstData.ReturnMessage.SERVICE_GET_ERROR;
                return false;
            }
        }
    }
}
