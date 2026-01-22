using DataAccess.Interface;
using DataAccess.Interfaces;
using DbObject;
using FluentValidation;
using DbRoute = DbObject.Route;

namespace RobotControlServer.Validators.RestApi.CRUD
{
    public class RouteValidator : BaseCRUDValidator<DbRoute>
    {
        private readonly IBaseDA<Map> _mapDa;
        private readonly IBaseDA<Point> _pointDa;
        public RouteValidator(IDbManagement dbManagement, IBaseDA<Map> mapDa, IBaseDA<Point> pointDa) : base(dbManagement)
        {
            _mapDa = mapDa;
            _pointDa = pointDa;
        }

        public override void InitRules()
        {
            RuleFor(x => x.MapId).NotNull().WithMessage("'mapId' must not be null");
            RuleFor(x => x.FromPointId).NotNull().WithMessage("'fromPointId' must not be null");
            RuleFor(x => x.ToPointId).NotNull().WithMessage("'toPointId' must not be null");
            RuleFor(x => x).Custom((route, context) =>
            {
                if (!route.MapId.HasValue || !route.FromPointId.HasValue || !route.ToPointId.HasValue)
                {
                    return;
                }

                using (var connection = _dbManagement.GetConnection())
                {
                    var map = _mapDa.Query(new Map { Id = route.MapId }, connection)?.FirstOrDefault();
                    if (map == null)
                    {
                        context.AddFailure("'mapId' is not found in database");
                        return;
                    }

                    var fromPoint = _pointDa.Query(new Point { Id = route.FromPointId }, connection)?.FirstOrDefault();
                    if (fromPoint == null)
                    {
                        context.AddFailure("'fromPointId' is not found in database");
                    }

                    var toPoint = _pointDa.Query(new Point { Id = route.ToPointId }, connection)?.FirstOrDefault();
                    if (toPoint == null)
                    {
                        context.AddFailure("'toPointId' is not found in database");
                    }

                    if (fromPoint != null && fromPoint.MapId != route.MapId)
                    {
                        context.AddFailure("'fromPointId' does not belong to selected map");
                    }

                    if (toPoint != null && toPoint.MapId != route.MapId)
                    {
                        context.AddFailure("'toPointId' does not belong to selected map");
                    }
                }
            });

            base.InitRules();
        }
    }
}
