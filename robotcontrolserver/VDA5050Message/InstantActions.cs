using System;
using System.Collections.Generic;
using System.Text.Json.Serialization;
using VDA5050Message.Base;
namespace VDA5050Message
{
    public class InstantActions : VDA5050MessageBase
    {
        public int HeaderId { get; set; }

        public DateTime Timestamp { get; set; }

        public string Version { get; set; }

        public string Manufacturer { get; set; }

        public string SerialNumber { get; set; }

        public List<InstantAction> Actions { get; set; } = new();
    }

    public class InstantAction
    {
        public string ActionId { get; set; }

        public string ActionType { get; set; }

        public string? ActionDescription { get; set; }

        [JsonConverter(typeof(JsonStringEnumConverter))]
        public BlockingType BlockingType { get; set; }

        public List<ActionParameter>? ActionParameters { get; set; }
    }

}
