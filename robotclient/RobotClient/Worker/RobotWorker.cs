using LocalMemmory;
using MqttService.Interfaces;
using RosNodeWrapper;
using RosNodeWrapper.Interfaces;

#nullable enable
namespace RobotClient.Worker
{
    public class RobotWorker : BackgroundService
    {
        private readonly IMqttClientService _mqttService;
        private readonly IVDARosClient _vdaRosClient;


        public RobotWorker(IMqttClientService serviceController, IVDARosClient vdaRosClient)
        {
            _mqttService = serviceController;
            _vdaRosClient = vdaRosClient;
        }

        public override async Task StartAsync(CancellationToken cancellationToken)
        {
            ShareMemoryData.LoadXmlConfig();
            await _mqttService.ConnectToBroker(cancellationToken);
            _vdaRosClient.StartSpinNode();
            _mqttService.StartReceiveMessage();   
            await base.StartAsync(cancellationToken);
            return;
        }
        protected override Task ExecuteAsync(CancellationToken stoppingToken)
        {

            return Task.CompletedTask;
        }
    }
}
