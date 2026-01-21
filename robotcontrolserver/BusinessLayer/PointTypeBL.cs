using BusinessLayer.Interfaces;
using DataAccess.Interface;
using DataAccess.Interfaces;
using DbObject;
using DbObject.Common;

namespace BusinessLayer
{
    public class PointTypeBL : BaseBL<PointType>, IPointTypeBL
    {
        public override string TableName => DatabaseEnum.TableName.PointType;
        public PointTypeBL(IBaseDA<PointType> baseDA, IDbManagement dbManagement) : base(baseDA, dbManagement)
        {
        }
    }
}
