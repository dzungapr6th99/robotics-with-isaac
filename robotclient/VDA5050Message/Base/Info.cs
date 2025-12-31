using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Text.Json.Serialization;
using System.Threading.Tasks;

namespace VDA5050Message.Base
{
    public class Info: VDA5050MessageBase
    {
        public string InfoType { get; set; }
        public List<InfoReference>? InfoReferences { get; set; }
        public string? InfoDescription { get; set; }

        [JsonConverter(typeof(JsonStringEnumConverter))]
        public InfoLevel InfoLevel { get; set; }

        public override void CreateWrapper() { }
        public override void GetDataWrapper(IntPtr prt) { }
    }

    public class InfoReference: VDA5050MessageBase
    {
        public string ReferenceKey { get; set; }
        public string ReferenceValue { get; set; }

        public override void CreateWrapper() { }
        public override void GetDataWrapper(IntPtr prt) { }
    }

    public enum InfoLevel
    {
        INFO,
        DEBUG
    }
}
