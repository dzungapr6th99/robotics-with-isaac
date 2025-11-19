using Microsoft.Extensions.Configuration;

namespace ConfigApp
{
    public static class ConfigData
    {
        public static class MqttConfig
        {
            public static string Endpoint = string.Empty;
            public static string ClientId = "Robot1";
            public static string AgvControl = "AgvControl";
        }
        public static void LoadConfig(IConfiguration configuration)
        {
            MqttConfig.Endpoint = configuration["Mqtt:Endpoint"]?.ToString() ?? "192.168.1.64:2010";
            MqttConfig.ClientId = configuration["SerialNumber"]?.ToString() ?? "Robot1";

        }
    }
}
