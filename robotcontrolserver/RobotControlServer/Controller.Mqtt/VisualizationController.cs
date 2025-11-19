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
        [MqttRoute("{robotId}/visualization")]
        public Task ProcessVisualization(string robotId, [FromPayload] Visualization visualization)
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
