using MQTTnet.Server;
namespace RobotControlServer.Workers
{
    public class MqttServiceWorker : BackgroundService
    {
        private readonly MqttServer _mqttServer;
        public MqttServiceWorker(MqttServer mqttServer)
        {
            _mqttServer = mqttServer;
        }

        public override Task StartAsync(CancellationToken cancellationToken)
        {
            return base.StartAsync(cancellationToken);
        }

        protected override  Task ExecuteAsync(CancellationToken stoppingToken)
        {
            _mqttServer.StartAsync();
            return Task.CompletedTask;

        }

    }
}
