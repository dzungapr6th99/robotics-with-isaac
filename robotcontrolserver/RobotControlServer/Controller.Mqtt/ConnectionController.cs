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
    public class ConnectionController : MqttBaseController
    {
        [MqttRoute("{interfaceName}/{majorVersion}/{manufacturer}/{serialNumber}/connection")]
        public Task ProcessConnection(string interfaceName, string majorVersion, string manufacturer, string serialNumber, [FromPayload] Connection connection)
        {
            CommonLog.logApi.Info($"Receive connection from {serialNumber}");

            return Ok();

        }
    }
}
