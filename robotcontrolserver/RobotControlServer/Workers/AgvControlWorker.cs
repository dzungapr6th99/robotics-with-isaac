using MQTTnet.Server;
using RobotControl.Interfaces;

namespace RobotControlServer.Workers
{
    public class AgvControlWorker : BackgroundService
    {
        private readonly IAgvControl _agvControl;
        public AgvControlWorker(IAgvControl mqttServer)
        {
            _agvControl = mqttServer;
        }

        public override Task StartAsync(CancellationToken cancellationToken)
        {
            return base.StartAsync(cancellationToken);
        }

        protected override Task ExecuteAsync(CancellationToken stoppingToken)
        {
            _agvControl.ConnectToServer(stoppingToken);
            return Task.CompletedTask;

        }

    }
}
