
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Text.Json.Serialization;
using System.Threading.Tasks;

namespace VDA5050Message
{
    public class Connection : VDA5050MessageBase
    {
        public int HeaderId { get; set; }

        public DateTime Timestamp { get; set; }

        public string Version { get; set; }

        public string Manufacturer { get; set; }

        public string SerialNumber { get; set; }

        [JsonConverter(typeof(JsonStringEnumConverter))]
        public ConnectionState ConnectionState { get; set; }

        public override void CreateWrapper() { }
        public override void GetDataWrapper(IntPtr prt) { }
    }

    public enum ConnectionState
    {
        ONLINE,
        OFFLINE,
        CONNECTIONBROKEN
    }
}
