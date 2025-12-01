namespace LocalMemmory
{
    public static class ShareMemoryData
    {
        public static class RobotStatus
        {
            public static double X { get; set; } = 0;
            public static double Y { get; set; } = 0;
        }

        public static class RobotConfiguration
        {
            public static string InterfaceName { get; set; } = string.Empty;
            public static string MajorVersion { get; set; } = string.Empty;
            public static string Manufacturer { get; set; } = string.Empty;
            public static string SerialNumber { get; set; } = string.Empty;
        }

    }
}
