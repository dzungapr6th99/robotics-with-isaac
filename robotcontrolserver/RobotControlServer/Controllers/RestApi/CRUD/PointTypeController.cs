using BusinessLayer.Interfaces;
using DbObject;
using RobotControlServer.Validators;
using Microsoft.AspNetCore.Mvc;

namespace RobotControlServer.Controllers.RestApi.CRUD
{
    [Route("api1/pointType/")]
    public class PointTypeController : CRUDBaseController<PointType>
    {
        public PointTypeController(IBaseBL<PointType> baseBL, BaseCRUDValidator<PointType>? validator = null) : base(baseBL, validator)
        {

        }
    }
}
