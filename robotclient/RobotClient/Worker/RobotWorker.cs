using MqttService.Interfaces;
using RosNodeWrapper;

#nullable enable
namespace RobotClient.Worker
{
    public class RobotWorker : BackgroundService
    {
        private IMqttClientService _mqttService;



        public RobotWorker(IMqttClientService serviceController)
        {
            _mqttService = serviceController;
        }

        public override async Task StartAsync(CancellationToken cancellationToken)
        {
#if !DEBUG
            VDARosClient.InitEnviroment();
            await _mqttService.ConnectToBroker();
            _mqttService.StartReceiveMessage();
#endif
            await base.StartAsync(cancellationToken);
            return;
        }
        protected override Task ExecuteAsync(CancellationToken stoppingToken)
        {

            return Task.CompletedTask;
        }
    }
}
