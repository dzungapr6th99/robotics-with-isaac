using FluentValidation;
using VDA5050Message;

namespace RobotControlServer.Validators.Mqtt
{
    public class OrderValidator:BaseMqttValidator<Order>
    {
        public OrderValidator() { }

        public override void InitRules()
        {
            RuleFor(order => order.SerialNumber).NotNull().NotEmpty();
            RuleFor(order => order.OrderId).NotNull().NotEmpty();
            RuleFor(order => order.Version).NotNull().NotEmpty();
                
            base.InitRules();
        }
    }
}
