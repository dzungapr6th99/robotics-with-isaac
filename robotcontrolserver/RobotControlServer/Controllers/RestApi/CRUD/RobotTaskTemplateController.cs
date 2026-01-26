using BusinessLayer.Interfaces;
using DbObject;
using Microsoft.AspNetCore.Mvc;
using RobotControlServer.Validators;

namespace RobotControlServer.Controllers.RestApi.CRUD
{
    [Route("api1/robottasktemplate/")]
    public class RobotTaskTemplateController : CRUDBaseController<RobotTaskTemplate>
    {
        public RobotTaskTemplateController(IBaseBL<RobotTaskTemplate> baseBL, BaseCRUDValidator<RobotTaskTemplate>? validator = null) : base(baseBL, validator)
        {

        }
    }
}
