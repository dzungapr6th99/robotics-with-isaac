using CommonLib;
using Entity.Api;
using LocalMemmory;
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
            bool changeConfig = ShareMemoryData.ChangeVDAConfig(request);
            if (changeConfig)
            {
                SettingResponse response = new SettingResponse()
                {
                    Code = "1",
                    Message = "Success"
                };
                return Ok(response);
            }
            else
            {
                SettingResponse response = new SettingResponse()
                {
                    Code = "-999",
                    Message = "Can not setting config now, please try later"
                };
                return BadRequest(response);
            }
        }

    }
}
