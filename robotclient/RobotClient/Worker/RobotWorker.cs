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

        public override Task StartAsync(CancellationToken cancellationToken)
        {
            VDARosClient.InitEnviroment();
            return base.StartAsync(cancellationToken);
        }
        protected override Task ExecuteAsync(CancellationToken stoppingToken)
        {

            return Task.CompletedTask;
        }
    }
}
