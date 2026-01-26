using BusinessLayer;
using BusinessLayer.Interfaces;
using ConfigApp;
using DataAccess;
using DataAccess.Helper;
using DataAccess.Interface;
using DataAccess.Interfaces;
using DbObject;
using MQTTnet.AspNetCore;
using MQTTnet.AspNetCore.Routing;
using MQTTnet.Server;
using RobotControl;
using RobotControl.Interfaces;
using RobotControlServer.Controllers.Mqtt;
using RobotControlServer.Controllers.RestApi.CRUD;
using RobotControlServer.Validators;
using RobotControlServer.Validators.RestApi.CRUD;
using RobotControlServer.Workers;
using System.Net;
using DbMap = DbObject.Map;
using DbRoute = DbObject.Route;

namespace RobotServer.Startups
{
    public static class Startup
    {
        public static void InitServices(IServiceCollection services)
        {
            services.AddCors();
            services.AddMqttConnectionHandler();
            DataAccessInjection(services);
            BusinessLayerInjection(services);
            ValidationInjection(services);
            //ControllerInjection(services);

            var connectionAssembly = typeof(ConnectionController).Assembly;

            services.AddHostedMqttServer(mqttServer => mqttServer.WithDefaultEndpoint().WithDefaultEndpointBoundIPAddress(IPAddress.Any).WithDefaultEndpointPort(ConfigData.PortMqtt))
                    .AddMqttConnectionHandler()
                    .AddConnections()
                    .AddMqttControllers([connectionAssembly]);
            services.AddSingleton<IAgvControl, AgvControl>();
            services.AddHostedService<MqttServiceWorker>();

        }

        public static void DataAccessInjection(IServiceCollection services)
        {
            services.AddSingleton<IDbManagement>(_ => new DbManagement(ConfigData.ConnectionString));
            
            services.AddSingleton<IBaseDA<DbMap>, MapDA>();
            services.AddSingleton<IBaseDA<Point>, PointDA>();
            services.AddSingleton<IBaseDA<PointType>, PointTypeDA>();
            services.AddSingleton<IBaseDA<DbRoute>, RouteDA>();
            services.AddSingleton<IBaseDA<Robot>, RobotDA>();
            services.AddSingleton<IBaseDA<RobotType>, RobotTypeDA>();
            services.AddSingleton<IBaseDA<RobotTask>, RobotTaskDA>();
            services.AddSingleton<IBaseDA<RobotTaskTemplate>, RobotTaskTemplateDA>();
        }

        public static void BusinessLayerInjection(IServiceCollection services)
        {
            services.AddSingleton<IBaseBL<DbMap>, MapBL>();
            services.AddSingleton<IBaseBL<Point>, PointBL>();
            services.AddSingleton<IBaseBL<PointType>, PointTypeBL>();
            services.AddSingleton<IBaseBL<DbRoute>, RouteBL>();
            services.AddSingleton<IBaseBL<Robot>, RobotBL>();
            services.AddSingleton<IBaseBL<RobotType>, RobotTypeBL>();
            services.AddSingleton<IBaseBL<RobotTask>, RobotTaskBL>();
            services.AddSingleton<IBaseBL<RobotTaskTemplate>, RobotTaskTemplateBL>();

            services.AddSingleton<IMapBL, MapBL>();
            services.AddSingleton<IPointBL, PointBL>();
            services.AddSingleton<IPointTypeBL, PointTypeBL>();
            services.AddSingleton<IRouteBL, RouteBL>();
            services.AddSingleton<IRobotBL, RobotBL>();
            services.AddSingleton<IRobotTypeBL, RobotTypeBL>();
            services.AddSingleton<IRobotTaskBL, RobotTaskBL>();
            services.AddSingleton<IRobotTaskTemplateBL, RobotTaskTemplateBL>();

        }

        public static void ValidationInjection(IServiceCollection services)
        {
            services.AddSingleton<BaseCRUDValidator<Point>, PointValidator>();
            services.AddSingleton<BaseCRUDValidator<DbRoute>, RouteValidator>();
            services.AddSingleton<BaseCRUDValidator<Robot>, RobotValidator>();
            services.AddSingleton<BaseCRUDValidator<RobotTaskTemplate>, RobotTaskTemplateValidator>();
        }

        public static void ControllerInjection(IServiceCollection services)
        {
            services.AddScoped<OrderController>();
            services.AddScoped<VisualizationController>();
            services.AddScoped<ConnectionController>();
            services.AddScoped<InstantActionsController>();
            services.AddScoped<StateController>();
            services.AddScoped<FactsheetController>();
            services.AddScoped<PointController>();
            services.AddScoped<MapController>();
            services.AddScoped<RouteController>();
            services.AddScoped<RobotController>();
            services.AddScoped<RobotTaskController>();
            services.AddScoped<RobotTaskTemplateController>();
            services.AddScoped<PointTypeController>();
            services.AddScoped<RobotTypeController>();
        }
    }
}
