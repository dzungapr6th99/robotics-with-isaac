using BusinessLayer.Interfaces;
using DbObject;
using RobotControlServer.Validators;

namespace RobotControlServer.Controllers.RestApi.CRUD
{
    public class PointController : CRUDBaseController<Point>
    {
        public PointController(IBaseBL<Point> baseBL, BaseCRUDValidator<Point> validator = null) : base(baseBL, validator)
        {

        }
    }
}
