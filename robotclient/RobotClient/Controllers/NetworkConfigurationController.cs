using Entity.Api;
using FluentValidation;
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
            }

            return Ok(response);
        }
    }
}
