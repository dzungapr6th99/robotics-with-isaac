using BusinessLayer.Interfaces;
using DbObject;
using RobotControlServer.Validators;
using Microsoft.AspNetCore.Mvc;

namespace RobotControlServer.Controllers.RestApi.CRUD
{
     [Route("api1/map/")]
    public class MapController : CRUDBaseController<Map>
    {
        public MapController(IBaseBL<Map> baseBL, BaseCRUDValidator<Map>? validator = null) : base(baseBL, validator)
        {

        }
    }
}
