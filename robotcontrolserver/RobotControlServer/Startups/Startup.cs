using MQTTnet.AspNetCore;
using ConfigApp;
using MQTTnet.AspNetCore.Routing;
using MQTTnet.Server;
using System.Net;
using RobotControlServer.Controller.Mqtt;
using System.Xml.Linq;
using VDA5050Message.Base;
using RobotControlServer.Workers;
namespace RobotServer.Startups
{
    public static class Startup
    {
        public static void InitServices(IServiceCollection services)
        {
            services.AddCors();
            services.AddMqttConnectionHandler();
          
            //Register Controller
            services.AddScoped<OrderController>();
            services.AddScoped<VisualizationController>();
            services.AddScoped<ConnectionController>();
            services.AddScoped<InstantActionsController>();
            services.AddScoped<StateController>();
            services.AddScoped<FactsheetController>();

            var orderAssembly = typeof(OrderController).Assembly;
            var connectionAssembly = typeof(ConnectionController).Assembly;

            services.AddHostedMqttServer(mqttServer => mqttServer.WithDefaultEndpoint().WithDefaultEndpointBoundIPAddress(IPAddress.Any).WithDefaultEndpointPort(ConfigData.PortMqtt))
                    .AddMqttConnectionHandler()
                    .AddConnections()
                    .AddMqttControllers([connectionAssembly]);
            services.AddHostedService<MqttServiceWorker>();
        }
    }
}
