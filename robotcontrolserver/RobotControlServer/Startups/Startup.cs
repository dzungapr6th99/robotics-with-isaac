using MQTTnet.AspNetCore;
using ConfigApp;
using MQTTnet.AspNetCore.Routing;
using MQTTnet.Server;
using System.Net;
using System.Xml.Linq;
using VDA5050Message.Base;
using RobotControlServer.Workers;
using RobotControlServer.Controllers.Mqtt;
namespace RobotServer.Startups
{
    public static class Startup
    {
        public static void InitServices(IServiceCollection services)
        {
            services.AddCors();
            services.AddMqttConnectionHandler();
            //Register DataAccess
            DataAccessInjection(services);
            //Register BusinessLayer
            BusinessLayerInjection(services);
            //Register Validation
            ValidationInjection(services);
            //Register Controller
            ControllerInjection(services);
            var orderAssembly = typeof(OrderController).Assembly;
            var connectionAssembly = typeof(ConnectionController).Assembly;

            services.AddHostedMqttServer(mqttServer => mqttServer.WithDefaultEndpoint().WithDefaultEndpointBoundIPAddress(IPAddress.Any).WithDefaultEndpointPort(ConfigData.PortMqtt))
                    .AddMqttConnectionHandler()
                    .AddConnections()
                    .AddMqttControllers([connectionAssembly]);
            services.AddHostedService<MqttServiceWorker>();
        }


        public static void DataAccessInjection(IServiceCollection services)
        {

        }

        public static void BusinessLayerInjection(IServiceCollection services)
        {

        }

        public static void ValidationInjection(IServiceCollection services)
        {

        }
        public static void ControllerInjection(IServiceCollection services)
        {
            services.AddScoped<OrderController>();
            services.AddScoped<VisualizationController>();
            services.AddScoped<ConnectionController>();
            services.AddScoped<InstantActionsController>();
            services.AddScoped<StateController>();
            services.AddScoped<FactsheetController>();
        }
    }
}
