using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Text.Json.Serialization;
using System.Threading.Tasks;

namespace VDA5050Message.Base
{
    public class Map: VDA5050MessageBase
    {
        public string MapId { get; set; }
        public string MapVersion { get; set; }
        public string? MapDescription { get; set; }

        [JsonConverter(typeof(JsonStringEnumConverter))]
        public MapStatus MapStatus { get; set; }

        public override void CreateWrapper() { }
        public override void GetDataWrapper(IntPtr prt) { }
    }

}
