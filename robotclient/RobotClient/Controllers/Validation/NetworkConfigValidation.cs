using Entity.Api;
using FluentValidation;

namespace RobotClient.Controllers.Validation
{
    public class NetworkConfigValidation : AbstractValidator<MqttSettingRequest>
    {
        public NetworkConfigValidation()
        {
            RuleFor(x => x.Ip).NotNull().NotEmpty().WithErrorCode("-100_001_001").WithMessage("IP is not null or empty");
            RuleFor(x => x.Port).NotNull().NotEmpty().WithErrorCode("-100_001_001").WithMessage("Port is not null or empty");
        }
    }
}
