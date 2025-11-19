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
        [MqttRoute("{robotId}/instantActions")]
        public Task ProcessInstantsAction(string robotId, [FromPayload] InstantActions instantActions)
        {
            CommonLog.logApi.Info($"Receive instantActions for robot {robotId}");

            return Ok();

        }
    }
}
