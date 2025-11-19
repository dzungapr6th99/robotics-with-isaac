using CommonLib;
using MQTTnet.AspNetCore.Routing;
using MQTTnet.AspNetCore.Routing.Attributes;
using VDA5050Message;

namespace RobotControlServer.Controller.Mqtt
{
    [MqttController]
    [MqttRoute("")]
    public class FactsheetController : MqttBaseController
    {
        public FactsheetController() { }
        [MqttRoute("{robotId}/facsheet")]
        public Task ProcessFactsheet(string robotId,[FromPayload] Factsheet factsheet)
        {
            CommonLog.logApi.Info("Receive instantActions from {0}", ClientId);
            if (!string.IsNullOrEmpty(factsheet.SerialNumber))
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
