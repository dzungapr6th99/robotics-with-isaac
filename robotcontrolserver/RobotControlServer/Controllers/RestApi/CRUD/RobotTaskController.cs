using BusinessLayer.Interfaces;
using DbObject;
using Microsoft.AspNetCore.Mvc;
using RobotControlServer.Validators;

namespace RobotControlServer.Controllers.RestApi.CRUD
{
    [Route("api1/robotTask/")]
    public class RobotTaskController : CRUDBaseController<RobotTask>
    {
        public RobotTaskController(IBaseBL<RobotTask> baseBL, BaseCRUDValidator<RobotTask>? validator = null) : base(baseBL, validator)
        {

        }
    }
}
