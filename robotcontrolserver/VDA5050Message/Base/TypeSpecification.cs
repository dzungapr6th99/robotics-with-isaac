using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Text.Json.Serialization;
using System.Threading.Tasks;

namespace VDA5050Message.Base
{
    public class TypeSpecification
    {
        public string SeriesName { get; set; }
        public string? SeriesDescription { get; set; }

        [JsonConverter(typeof(JsonStringEnumConverter))]
        public AgvKinematic AgvKinematic { get; set; }

        [JsonConverter(typeof(JsonStringEnumConverter))]
        public AgvClass AgvClass { get; set; }

        public double MaxLoadMass { get; set; }
        public List<LocalizationType> LocalizationTypes { get; set; }
        public List<NavigationType> NavigationTypes { get; set; }
    }

    public enum AgvKinematic
    {
        DIFF,
        OMNI,
        THREEWHEEL
    }

    public enum AgvClass
    {
        FORKLIFT,
        CONVEYOR,
        TUGGER,
        CARRIER
    }

    public enum LocalizationType
    {
        NATURAL,
        REFLECTOR,
        RFID,
        DMC,
        SPOT,
        GRID
    }

    public enum NavigationType
    {
        PHYSICAL_LINE_GUIDED,
        VIRTUAL_LINE_GUIDED,
        AUTONOMOUS
    }
}
