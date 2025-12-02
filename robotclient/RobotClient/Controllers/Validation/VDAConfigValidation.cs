using Entity.Api;
using FluentValidation;
using FluentValidation.Validators;
namespace RobotClient.Controllers.Validation
{
    public class VDAConfigValidation : AbstractValidator<VDASettingsRequest>
    {
        public VDAConfigValidation()
        {
            RuleFor(x=> x.InterfaceName).NotNull().NotEmpty().WithErrorCode("-100_000_001").WithMessage("InterfaceName is not null or empty");
            RuleFor(x => x.MajorVersion).NotNull().NotEmpty().WithErrorCode("-100_000_002").WithMessage("MajorVersion is not null or empty");
            RuleFor(x => x.Manufacturer).NotNull().NotEmpty().WithErrorCode("-100_000_003").WithMessage("Manufacturer is not null or empty");
            RuleFor(x => x.SerialNumber).NotNull().NotEmpty().WithErrorCode("-100_000_004").WithMessage("SerialNumber is not null or empty");
            
        }
    }
}
