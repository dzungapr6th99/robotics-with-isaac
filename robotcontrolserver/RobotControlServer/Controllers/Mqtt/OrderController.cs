using MQTTnet;
using MQTTnet.AspNetCore;
using MQTTnet.Extensions;
using CommonLib;
using MQTTnet.AspNetCore.Routing;
using VDA5050Message;
using MQTTnet.AspNetCore.Routing.Attributes;
namespace RobotControlServer.Controllers.Mqtt
{
    [MqttController]
    [MqttRoute("")]
    public class OrderController : MqttBaseController
    {
        [MqttRoute("{interfaceName}/{majorVersion}/{manufacturer}/{serialNumber}/order")]
        public Task ProcessOrder(string interfaceName, string majorVersion, string manufacturer, string serialNumber, [FromPayload] Order order)
        {
            CommonLog.logApi.Info($"Receive order from {serialNumber}");

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
