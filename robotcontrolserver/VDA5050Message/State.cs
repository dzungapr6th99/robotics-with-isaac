using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Text.Json.Serialization;
using System.Threading.Tasks;
using VDA5050Message.Base;
namespace VDA5050Message
{
    public class State : VDA5050MessageBase
    {
        public int HeaderId { get; set; }
        public DateTime Timestamp { get; set; }
        public string Version { get; set; }
        public string Manufacturer { get; set; }
        public string SerialNumber { get; set; }
        public List<Map>? Maps { get; set; }
        
        public string? OrderId { get; set; }
        public int OrderUpdateId { get; set; }
        public string? ZoneSetId { get; set; }
        public string? LastNodeId { get; set; }
        public int LastNodeSequenceId { get; set; }
        public bool Driving { get; set; }
        public bool? Paused { get; set; }
        public bool? NewBaseRequest { get; set; }
        public double? DistanceSinceLastNode { get; set; }

        [JsonConverter(typeof(JsonStringEnumConverter))]
        public OperatingMode OperatingMode { get; set; }

        public List<NodeState> NodeStates { get; set; }
        public List<EdgeState> EdgeStates { get; set; }

        public AgvPosition? AgvPosition { get; set; }
        public Velocity? Velocity { get; set; }
        public List<Load>? Loads { get; set; }
        public List<ActionState> ActionStates { get; set; }
        public BatteryState BatteryState { get; set; }
        public List<Error> Errors { get; set; }
        public List<Info>? Information { get; set; }
        public SafetyState SafetyState { get; set; }
    }


  

}
