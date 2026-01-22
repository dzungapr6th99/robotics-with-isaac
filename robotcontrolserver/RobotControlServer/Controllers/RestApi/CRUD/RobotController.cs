using BusinessLayer.Interfaces;
using DbObject;
using Microsoft.AspNetCore.Mvc;
using RobotControlServer.Validators;

namespace RobotControlServer.Controllers.RestApi.CRUD
{

    [Route("api1/robot/")]
    public class RobotController : CRUDBaseController<Robot>
    {
        public RobotController(IBaseBL<Robot> baseBL, BaseCRUDValidator<Robot>? validator = null) : base(baseBL, validator)
        {

        }
    }
}
