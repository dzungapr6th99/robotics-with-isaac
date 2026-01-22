using Microsoft.Extensions.Configuration;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Security.Cryptography.X509Certificates;
using System.Text;
using System.Threading.Tasks;

namespace ConfigApp
{
    public static class ConfigData
    {
      

        public static class MqttClientConfig
        {

            public static string Endpoint = "127.0.0.1:3006";
            public static string AgvControl = "AgvControl";
        }
        public static int PortMqtt { get; set; } = 2010;
        public static int RingBufferSize = 1024;
        public static string Manufacturer = "Dhhhh-team";
        public static string Version = "1.0";
        public static string ConnectionString { get; set; } = "Data Source=robotcontrol.db;Version=3;";
        public static void InitConfig(IConfiguration configuration)
        {
            bool parse = int.TryParse(configuration["PortMqtt"]?.ToString(), out int value);
            if (parse)
            {
                PortMqtt = value;
            }
            else
            {
                PortMqtt = 2010;
            }

            string? configuredConnection = configuration.GetConnectionString("RobotDatabase") ?? configuration["ConnectionString"];
            if (!string.IsNullOrWhiteSpace(configuredConnection))
            {
                ConnectionString = configuredConnection;
            }
        }
    }
}
