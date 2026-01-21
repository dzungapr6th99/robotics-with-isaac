
namespace ShareMemoryData
{
    public class RobotStatus
    {
        public string SerialNumber { get; set; } = string.Empty;
        public double VelocityX { get; set; }
        public double VelocityY { get; set; }
        public double CoordinateX { get; set; }
        public double CoordinateY { get; set; }
        public string DoingTask { get; set; } = string.Empty;
        public string Status { get; set; } = string.Empty;
        public int MaxCapacity { get; set; }
        public int CurrentCapacity { get; set; }
        public int RemainCapacity { get; set; }
    }
}
