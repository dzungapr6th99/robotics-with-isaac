using DataAccess.Interface;
using DataAccess.Interfaces;
using DbObject;
using FluentValidation;

namespace RobotControlServer.Validators.RestApi.CRUD
{
    public class RobotTaskTemplateValidator : BaseCRUDValidator<RobotTaskTemplate>
    {
        private readonly IBaseDA<Map> _mapDa;
        private readonly IBaseDA<Point> _pointDa;
        private readonly IBaseDA<RobotType> _robotTypeDa;

        public RobotTaskTemplateValidator(IDbManagement dbManagement, IBaseDA<Map> mapDa, IBaseDA<Point> pointDa, IBaseDA<RobotType> robotTypeDa) : base(dbManagement)
        {
            _mapDa = mapDa;
            _pointDa = pointDa;
            _robotTypeDa = robotTypeDa;
        }

        public override void InitRules()
        {
            RuleFor(x => x.MapId).NotNull().WithMessage("'mapId' must not be null");
            RuleFor(x => x.FromPointId).NotNull().WithMessage("'fromPointId' must not be null");
            RuleFor(x => x.ToPointId).NotNull().WithMessage("'toPointId' must not be null");
            RuleFor(x => x.RobotTypeId).NotNull().WithMessage("'robotTypeId' must not be null");

            RuleFor(x => x).Custom((template, context) =>
            {
                using var connection = _dbManagement.GetConnection();
                if (template.MapId.HasValue)
                {
                    var map = _mapDa.Query(new Map { Id = template.MapId }, connection)?.FirstOrDefault();
                    if (map == null)
                    {
                        context.AddFailure("'mapId' not found in database");
                    }
                }
                if (template.FromPointId.HasValue)
                {
                    var point = _pointDa.Query(new Point { Id = template.FromPointId }, connection)?.FirstOrDefault();
                    if (point == null)
                    {
                        context.AddFailure("'fromPointId' not found in database");
                    }
                    else if (template.MapId.HasValue && point.MapId != template.MapId)
                    {
                        context.AddFailure("'fromPointId' does not belong to selected map");
                    }
                }
                if (template.ToPointId.HasValue)
                {
                    var point = _pointDa.Query(new Point { Id = template.ToPointId }, connection)?.FirstOrDefault();
                    if (point == null)
                    {
                        context.AddFailure("'toPointId' not found in database");
                    }
                    else if (template.MapId.HasValue && point.MapId != template.MapId)
                    {
                        context.AddFailure("'toPointId' does not belong to selected map");
                    }
                }
                if (template.RobotTypeId.HasValue)
                {
                    var robotType = _robotTypeDa.Query(new RobotType { Id = template.RobotTypeId }, connection)?.FirstOrDefault();
                    if (robotType == null)
                    {
                        context.AddFailure("'robotTypeId' not found in database");
                    }
                }
            });

            base.InitRules();
        }
    }
}
