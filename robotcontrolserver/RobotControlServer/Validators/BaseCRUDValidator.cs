using DataAccess.Interface;
using FluentValidation;
using FluentValidation.Results;

namespace RobotControlServer.Validators
{
    public class BaseCRUDValidator<T> :AbstractValidator<T> where T : class
    {
        public readonly IDbManagement _dbManagement;
        public BaseCRUDValidator(IDbManagement dbManagement)
        {
            _dbManagement = dbManagement;
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
