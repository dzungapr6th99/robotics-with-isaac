using DataAccess.Interface;
using FluentValidation;
using FluentValidation.Results;
using VDA5050Message;

namespace RobotControlServer.Validators
{
    public class BaseMqttValidator<T> : AbstractValidator<T> where T : VDA5050MessageBase
    {
        public BaseMqttValidator()
        {
            
        }
        public override ValidationResult Validate(ValidationContext<T> context)
        {

            return base.Validate(context);
        }

        public virtual void InitRules()
        {

        }
    }
}
