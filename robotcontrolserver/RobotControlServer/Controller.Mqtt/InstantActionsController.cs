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
    public class InstantActionsController : MqttBaseController
    {
        [MqttRoute("{interfaceName}/{majorVersion}/{manufacturer}/{serialNumber}/instantActions")]
        public Task ProcessInstantsAction(string interfaceName, string majorVersion, string manufacturer, string serialNumber, [FromPayload] InstantActions instantActions)
        {
            CommonLog.logApi.Info($"Receive instantActions for robot {serialNumber}");

            return Ok();

        }
    }
}
