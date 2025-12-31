using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Text.Json.Serialization;
using System.Threading.Tasks;

namespace VDA5050Message.Base
{
    public class Info
    {
        public string InfoType { get; set; }
        public List<InfoReference>? InfoReferences { get; set; }
        public string? InfoDescription { get; set; }

        [JsonConverter(typeof(JsonStringEnumConverter))]
        public InfoLevel InfoLevel { get; set; }
    }

    public class InfoReference
    {
        public string ReferenceKey { get; set; }
        public string ReferenceValue { get; set; }
    }

    public enum InfoLevel
    {
        INFO,
        DEBUG
    }
}
