using ApiObject.CRUD;
using BusinessLayer.Interfaces;
using DbObject;
using Microsoft.AspNetCore.Mvc;
using RobotControlServer.Validators;
using System.Security.Cryptography.X509Certificates;

namespace RobotControlServer.Controllers.RestApi.CRUD
{
    [Route("api1/robotTask/")]
    public class RobotTaskController : CRUDBaseController<RobotTask>
    {
        private readonly IRobotTaskCoordinator _robotTaskCoordinator;
        public RobotTaskController(IBaseBL<RobotTask> baseBL, IRobotTaskCoordinator robotTaskCoordinator, BaseCRUDValidator<RobotTask>? validator = null) : base(baseBL, validator)
        {
            _robotTaskCoordinator = robotTaskCoordinator;
        }
        [HttpPost("execute-task")]
        public async Task<IActionResult> ExecuteTask(int robotTaskId)
        {
            RobotTask? robotTask = _baseBL.GetById(robotTaskId, out int returnCode, out string returnMessage);
            if (robotTask == null)
            {
                return BadRequest(new
                {
                    ReturnCode = returnCode,
                    ReturnMessage = returnMessage
                });
            }
            _robotTaskCoordinator.Enqueue(robotTask);
            return Ok(new {
                ReturnCode = 1,
                ReturnMessage = "success"
            });


        }
    }
}
