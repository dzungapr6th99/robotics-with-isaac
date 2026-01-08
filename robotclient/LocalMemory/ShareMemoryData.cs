using CommonLib;
using ConfigApp;
using Entity;
using Entity.Api;

namespace LocalMemmory
{
    public static class ShareMemoryData
    {
        public static class RobotStatus
        {
            public static double X { get; set; } = 0;
            public static double Y { get; set; } = 0;
        }

        public static VDA5050Configuration RobotConfiguration { get; set; } = new VDA5050Configuration();

        public static void LoadXmlConfig()
        {
            CommonLog.log.Info("Loading VDA5050 Configuration from XML file...");
            VDA5050Configuration? configuration = CommonFunc.LoadFromXmlFile<VDA5050Configuration>(ConfigData.PathStoreConfig);
            if (configuration == null)
            {
                RobotConfiguration.IP = "127.0.0.1";
                RobotConfiguration.Port = 3011;
                RobotConfiguration.InterfaceName = "ntd";
                RobotConfiguration.MajorVersion = "v2.1.0";
                RobotConfiguration.Manufacturer = "nthb";
                RobotConfiguration.SerialNumber = "Robot1";
                CommonFunc.SaveToXmlFile<VDA5050Configuration>(RobotConfiguration, ConfigData.PathStoreConfig);
            }
            else
            {
                RobotConfiguration = configuration;
            }
            CommonLog.log.Info("VDA5050 Configuration loaded successfully.");
        }

        public static string GetParentTopic()
        {
            return $"{RobotConfiguration.InterfaceName}/{RobotConfiguration.MajorVersion}/{RobotConfiguration.Manufacturer}/{RobotConfiguration.SerialNumber}";
        }

        public static bool ChangeVDAConfig(VDASettingsRequest request)
        {
            RobotConfiguration.InterfaceName = request.InterfaceName;
            RobotConfiguration.MajorVersion = request.MajorVersion;
            RobotConfiguration.Manufacturer = request.Manufacturer;
            RobotConfiguration.SerialNumber = request.SerialNumber;
            CommonFunc.SaveToXmlFile<VDA5050Configuration>(RobotConfiguration, ConfigData.PathStoreConfig);
            return true;
        }

        public static bool ChangeNetworkConfig(MqttSettingRequest request)
        {
            bool tryParse = int.TryParse(request.Port, out int value);
            if (tryParse)
            {
                RobotConfiguration.Port = value;
                RobotConfiguration.IP = request.Ip;
                CommonFunc.SaveToXmlFile<VDA5050Configuration>(RobotConfiguration, ConfigData.PathStoreConfig);
                return true;
            }
            else
            {
                return false;
            }
        }
    }
}
