using Entity.Api;
using Microsoft.AspNetCore.Http.HttpResults;
using Microsoft.AspNetCore.Mvc;

namespace RobotClient.Controllers
{
    [ApiController]
    [Route("vda-configuration")]
    public class VDAConfigurationController : ControllerBase
    {
        [HttpPost("topic")]
        public async Task<IActionResult> ConfigVDA([FromBody] VDASettingsRequest request)
        {
            SettingResponse response = new SettingResponse();
            return Ok(response);
        }


    }
}
