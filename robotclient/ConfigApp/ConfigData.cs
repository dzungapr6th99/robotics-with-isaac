using Microsoft.Extensions.Configuration;

namespace ConfigApp
{
    public static class ConfigData
    {
        public static string PathStoreConfig { get; set; } = string.Empty;
        public static string MqttClientId { get; set; } = string.Empty;
        public static string RosNamespace {  get; set; } = string.Empty;
        public static void LoadConfig(IConfiguration configuration)
        {
            PathStoreConfig = configuration["PathStoreConfig"]?.ToString() ?? string.Empty;
            MqttClientId = configuration["MqttClientId"]?.ToString() ?? string.Empty;
            RosNamespace = configuration["RobotNameSpace"]?.ToString() ?? string.Empty;
        }
    }
}
