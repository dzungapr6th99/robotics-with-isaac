using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Text.Json.Serialization;
using System.Threading.Tasks;

namespace VDA5050Message.Base
{
    public class Action
    {
        public string ActionType { get; set; }

        public string ActionId { get; set; }

        public string? ActionDescription { get; set; }
        [JsonConverter(typeof(JsonStringEnumConverter))]
        public BlockingType BlockingType { get; set; }

        public List<ActionParameter>? ActionParameters { get; set; }
    }
    public class ActionParameter
    {
        [JsonPropertyName("key")]
        public string Key { get; set; }

        [JsonPropertyName("value")]
        public object Value { get; set; }
    }

    public class ActionState
    {
        public string ActionId { get; set; }
        public string? ActionType { get; set; }
        public string? ActionDescription { get; set; }

        [JsonConverter(typeof(JsonStringEnumConverter))]
        public ActionStatus ActionStatus { get; set; }

        public string? ResultDescription { get; set; }
    }

    public enum ActionStatus
    {
        WAITING,
        INITIALIZING,
        RUNNING,
        PAUSED,
        FINISHED,
        FAILED
    }
}
