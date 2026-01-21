using CommonLib;
using MQTTnet.AspNetCore.Routing;
using MQTTnet.AspNetCore.Routing.Attributes;
using VDA5050Message;

namespace RobotControlServer.Controllers.Mqtt
{
    [MqttController]
    [MqttRoute("")]
    public class FactsheetController : MqttBaseController
    {
        public FactsheetController() { }

        [MqttRoute("{interfaceName}/{majorVersion}/{manufacturer}/{serialNumber}/factsheet")]
        public Task ProcessFactsheet(string interfaceName, string majorVersion, string manufacturer, string serialNumber, [FromPayload] Factsheet factsheet)
        {
            CommonLog.logApi.Info("Receive factsheet from {0}", serialNumber);
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
