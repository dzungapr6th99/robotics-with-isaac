namespace LocalMemmory
{
    public static class ShareMemoryData
    {
        /// <summary>
        /// Settings for robot, can be fix from api
        /// </summary>
        public static class RobotSettings
        {
            #region VDA5050 config
            public static string InterfaceName { get; set; } = "vda";
            public const string MajorVersion = "v2.1.0";
            public static string SerialNumber { get; set; } = "0";
            public static string Manufacturer { get; set; } = "dz";
            #endregion

            #region Network Config
            public static string Uri = "localhost";
            public static int Port = 2010;
            #endregion
        }
    }
}
