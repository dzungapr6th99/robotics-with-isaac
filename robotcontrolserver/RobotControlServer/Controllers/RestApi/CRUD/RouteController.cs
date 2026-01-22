using BusinessLayer.Interfaces;
using Microsoft.AspNetCore.Mvc;
using RobotControlServer.Validators;
using DbRoute = DbObject.Route;

namespace RobotControlServer.Controllers.RestApi.CRUD
{

    [Route("api1/route/")]
    public class RouteController : CRUDBaseController<DbRoute>
    {
        public RouteController(IBaseBL<DbRoute> baseBL, BaseCRUDValidator<DbRoute>? validator = null) : base(baseBL, validator)
        {

        }
    }
}
