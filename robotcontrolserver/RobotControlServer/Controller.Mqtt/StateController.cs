using CommonLib;
using MQTTnet.AspNetCore.Routing;
using MQTTnet.AspNetCore.Routing.Attributes;
using VDA5050Message;

namespace RobotControlServer.Controller.Mqtt
{
    [MqttController()]
    [MqttRoute("")]
    public class StateController : MqttBaseController
    {
        

        [MqttRoute("{interfaceName}/{majorVersion}/{manufacturer}/{serialNumber}/state")]
        public async Task ProcessState(string interfaceName, string majorVersion, string manufacturer, string serialNumber, [FromPayload] State state)
        {
            CommonLog.logApi.Info("Receive state from {0}", serialNumber);

            if (string.IsNullOrEmpty(state.SerialNumber))
            {
                await BadMessage();

            }
            else
            {
                // Process loop task if controller is available

                await Ok();
            }
        }
    }
}
