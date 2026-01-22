using BusinessLayer.Interfaces;
using DbObject;
using Microsoft.AspNetCore.Mvc;
using RobotControlServer.Validators;

namespace RobotControlServer.Controllers.RestApi.CRUD
{
     [Route("api1/point/")]
    public class PointController : CRUDBaseController<Point>
    {
        public PointController(IBaseBL<Point> baseBL, BaseCRUDValidator<Point>? validator = null) : base(baseBL, validator)
        {

        }
    }
}
