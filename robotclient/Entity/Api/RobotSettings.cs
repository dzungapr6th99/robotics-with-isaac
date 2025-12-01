namespace Entity.Api
{
    /// <summary>
    /// Settings for robot, can be fix from api
    /// </summary>
    public class RobotSettingsRequest
    {
        #region VDA5050 config
        public string? InterfaceName { get; set; }
        public string? MajorVersion {get;set;}
        public string? SerialNumber { get; set; }
        public string? Manufacturer { get; set; }
        #endregion

        #region Network Config
        public string Uri = "localhost";
        public int Port = 2010;
        #endregion
    }

    public class RobotSettingResponse
    {
        public int Code { get; set; }
        public string Message { get; set; } = string.Empty;
    }
}
