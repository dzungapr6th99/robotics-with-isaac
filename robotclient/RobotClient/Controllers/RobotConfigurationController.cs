using Microsoft.AspNetCore.Http.HttpResults;
using Microsoft.AspNetCore.Mvc;

namespace RobotClient.Controllers
{
    [ApiController]
    [Route("robot-configuration")]
    public class RobotConfigurationController : ControllerBase
    {
        [HttpPost("vda5050")]
        public Task<IActionResult> ConfigVDA()
        {
            return Ok(new object()
            {
                
            });
        }

        public Task<IActionResult> ConfigMqtt()
        {
            return Ok(new object());
        }

    }
}
