
using MqttService;
using MqttService.Interfaces;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RobotClient.Startups
{
    public static class StartupApp
    {
        public static void InitServices(IServiceCollection services)
        {
            services.AddSingleton<IMqttClientService, MqttClientService>();
        }
    }
}
