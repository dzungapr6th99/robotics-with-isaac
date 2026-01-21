using DataAccess.Interface;
using DataAccess.Interfaces;
using DbObject;
using FluentValidation;

namespace RobotControlServer.Validators.RestApi.CRUD
{
    public class PointValidator : BaseCRUDValidator<Point>
    {
        private readonly IBaseDA<Map> _mapBaseDA;
        public PointValidator(IDbManagement dbManagement, IBaseDA<Map> mapBaseDA) : base(dbManagement)
        {
            _mapBaseDA = mapBaseDA;
        }

        public override void InitRules()
        {
            RuleFor(x => x.MapId).NotNull().Must((mapId) =>
            {                
                using (var connection = _dbManagement.GetConnection())
                {
                    Map? map = _mapBaseDA.Query(new Map()
                    {
                        Id = mapId
                    }, connection)?.FirstOrDefault();
                    if (map == null)
                    {
                        return false;
                    }    
                }

                return true;
            }).WithMessage("'mapId' must not null or can not file Map with this Id in database");
            base.InitRules();
        }
    }
}
