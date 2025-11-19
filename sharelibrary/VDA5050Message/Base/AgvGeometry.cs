using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Text.Json.Serialization;
using System.Threading.Tasks;

namespace VDA5050Message.Base
{
    public class AgvGeometry
    {
        public List<WheelDefinition>? WheelDefinitions { get; set; }
        public List<Envelope2d>? Envelopes2d { get; set; }
        public List<Envelope3d>? Envelopes3d { get; set; }
    }

    public class WheelDefinition
    {
        [JsonConverter(typeof(JsonStringEnumConverter))]
        public WheelType Type { get; set; }

        public bool IsActiveDriven { get; set; }
        public bool IsActiveSteered { get; set; }
        public Position Position { get; set; }
        public double Diameter { get; set; }
        public double Width { get; set; }
        public double? CenterDisplacement { get; set; }
        public string? Constraints { get; set; }
    }

    public enum WheelType
    {
        DRIVE,
        CASTER,
        FIXED,
        MECANUM
    }

    public class Position
    {
        public double X { get; set; }
        public double Y { get; set; }
        public double? Theta { get; set; }
    }

    public class Envelope2d
    {
        public string Set { get; set; }
        public List<PolygonPoint> PolygonPoints { get; set; }
        public string? Description { get; set; }
    }

    public class PolygonPoint
    {
        public double X { get; set; }
        public double Y { get; set; }
    }

    public class Envelope3d
    {
        public string Set { get; set; }
        public string Format { get; set; }
        public object? Data { get; set; }  // Can be replaced with a specific type if known
        public string? Url { get; set; }
        public string? Description { get; set; }
    }
}
