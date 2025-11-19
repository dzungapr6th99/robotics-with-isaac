using MQTTnet;
using MQTTnet.AspNetCore;
using MQTTnet.Extensions;
using CommonLib;
using MQTTnet.AspNetCore.Routing;
using VDA5050Message;
using MQTTnet.AspNetCore.Routing.Attributes;
namespace RobotControlServer.Controller.Mqtt
{
    [MqttController]
    [MqttRoute("")]
    public class OrderController : MqttBaseController
    {
        [MqttRoute("{robotId}/order")]
        public Task ProcessOrder(string robotId,[FromPayload]Order order)
        {
            CommonLog.logApi.Info($"Receive order from {robotId}");

            if (!string.IsNullOrEmpty(order.OrderId))
            {
                return Ok();
            }
            else
            {
                return BadMessage();
            }

        }
    }
}
