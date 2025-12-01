using Entity.Api;
using Microsoft.AspNetCore.Http.HttpResults;
using Microsoft.AspNetCore.Mvc;

namespace RobotClient.Controllers
{
    [ApiController]
    [Route("robot-configuration")]
    public class RobotConfigurationController : ControllerBase
    {
        [HttpPost("vda5050-topic")]
        public async Task<IActionResult> ConfigVDA([FromBody] RobotSettingsRequest request)
        {
            SettingResponse response = new SettingResponse();
            return Ok(response);
        }

        [HttpPost("vda5050-network")]
        public async Task<IActionResult> ConfigMqtt([FromBody]MqttSettingRequest request)
        {
            SettingResponse response = new SettingResponse();
            return Ok(response);
        }

    }
}
