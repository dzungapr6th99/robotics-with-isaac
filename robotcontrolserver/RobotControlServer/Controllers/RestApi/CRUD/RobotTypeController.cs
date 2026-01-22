using BusinessLayer.Interfaces;
using DbObject;
using Microsoft.AspNetCore.Mvc;
using RobotControlServer.Validators;

namespace RobotControlServer.Controllers.RestApi.CRUD
{
    [Route("api1/robotType/")]
    public class RobotTypeController : CRUDBaseController<RobotType>
    {
        public RobotTypeController(IBaseBL<RobotType> baseBL, BaseCRUDValidator<RobotType>? validator = null) : base(baseBL, validator)
        {

        }
    }
}
