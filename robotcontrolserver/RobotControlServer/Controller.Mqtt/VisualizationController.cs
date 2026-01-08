using CommonLib;
using MQTTnet.AspNetCore.Routing;
using MQTTnet.AspNetCore.Routing.Attributes;
using VDA5050Message;

namespace RobotControlServer.Controller.Mqtt
{
    [MqttController()]
    [MqttRoute("")]
    public class VisualizationController : MqttBaseController
    {
        public VisualizationController() { }
        [MqttRoute("{interfaceName}/{majorVersion}/{manufacturer}/{serialNumber}/visualization")]
        public Task ProcessVisualization(string interfaceName, string majorVersion, string manufacturer, string serialNumber, [FromPayload] Visualization visualization)
        {

            CommonLog.logApi.Info("Receive visualization from {0} headerId {1} serialNumber {2}", ClientId, visualization.HeaderId, visualization.SerialNumber);
            if (!string.IsNullOrEmpty(visualization.SerialNumber))
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
