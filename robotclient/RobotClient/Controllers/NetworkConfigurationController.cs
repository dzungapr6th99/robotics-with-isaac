using Entity.Api;
using FluentValidation;
using LocalMemmory;
using Microsoft.AspNetCore.Mvc;

namespace RobotClient.Controllers
{
    [Route("network")]
    public class NetworkConfigurationController : ControllerBase
    {
        private readonly IValidator<MqttSettingRequest> _validator;

        public NetworkConfigurationController(IValidator<MqttSettingRequest> validator)
        {
            _validator = validator;
        }

        [HttpPost("mqtt")]
        public async Task<IActionResult> ConfigMqtt([FromBody] MqttSettingRequest request)
        {
            SettingResponse response = new SettingResponse();
            var validation = await _validator.ValidateAsync(request);
            if (!validation.IsValid)
            {
                response.Code =  validation.Errors[0].ErrorCode;
                response.Message = validation.Errors[0].ErrorMessage;
                return BadRequest(response);
            }

            bool changeNetwork = ShareMemoryData.ChangeNetworkConfig(request);
            if (changeNetwork)
            {
                response.Code = "1";
                response.Message = "Success";
                return Ok(response);
            }
            else
            {
                response.Code = "-1";
                response.Message = "Can not change network config now, please change it later";
                return BadRequest(response);
            }
        }
    }
}
