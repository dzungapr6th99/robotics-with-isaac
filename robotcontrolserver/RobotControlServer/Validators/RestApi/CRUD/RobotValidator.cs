using DataAccess.Interface;
using DataAccess.Interfaces;
using DbObject;
using FluentValidation;

namespace RobotControlServer.Validators.RestApi.CRUD
{
    public class RobotValidator : BaseCRUDValidator<Robot>
    {
        private readonly IBaseDA<Robot> _robotDa;

        public RobotValidator(IDbManagement dbManagement, IBaseDA<Robot> robotDa) : base(dbManagement)
        {
            _robotDa = robotDa;
        }

        public override void InitRules()
        {
            RuleFor(x => x.SerialNumber).NotEmpty().WithMessage("'serialNumber' must not be empty");
            RuleFor(x => x).Custom((robot, context) =>
            {
                if (string.IsNullOrWhiteSpace(robot.SerialNumber))
                {
                    return;
                }

                using (var connection = _dbManagement.GetConnection())
                {
                    var duplicated = _robotDa.Query(new Robot { SerialNumber = robot.SerialNumber }, connection)?.FirstOrDefault();
                    if (duplicated != null && duplicated.Id != robot.Id)
                    {
                        context.AddFailure("'serialNumber' already exists in database");
                    }
                }
            });

            base.InitRules();
        }
    }
}
