using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Text.Json.Serialization;
using System.Threading.Tasks;

namespace VDA5050Message.Base
{
    public class ProtocolFeatures
    {
        public List<OptionalParameter> OptionalParameters { get; set; }
        public List<AgvAction> AgvActions { get; set; }
    }

    public class OptionalParameter
    {
        public string Parameter { get; set; }

        [JsonConverter(typeof(JsonStringEnumConverter))]
        public SupportType Support { get; set; }

        public string? Description { get; set; }
    }

    public enum SupportType
    {
        SUPPORTED,
        REQUIRED
    }

    public class AgvAction
    {
        public string ActionType { get; set; }
        public string? ActionDescription { get; set; }
        public List<ActionScope> ActionScopes { get; set; }
        public List<ActionParameterSpec>? ActionParameters { get; set; }
        public string? ResultDescription { get; set; }
        public List<BlockingType>? BlockingTypes { get; set; }
    }

    public enum ActionScope
    {
        INSTANT,
        NODE,
        EDGE
    }

    public class ActionParameterSpec
    {
        public string Key { get; set; }

        [JsonConverter(typeof(JsonStringEnumConverter))]
        public ValueDataType ValueDataType { get; set; }

        public string? Description { get; set; }
        public bool? IsOptional { get; set; }
    }

    public enum ValueDataType
    {
        BOOL,
        NUMBER,
        INTEGER,
        FLOAT,
        STRING,
        OBJECT,
        ARRAY
    }

}
