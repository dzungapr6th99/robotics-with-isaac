
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
        [JsonIgnore(Condition = JsonIgnoreCondition.Never)]
        public override int HeaderId { get { return base.HeaderId; } set { base.HeaderId = value; } }

        public override DateTime Timestamp { get { return base.Timestamp; } set { base.Timestamp = value; } }
        [JsonIgnore(Condition = JsonIgnoreCondition.Never)]
        public override string Version { get { return base.Version; } set { base.Version = value; } }
        [JsonIgnore(Condition = JsonIgnoreCondition.Never)]
        public override string Manufacturer { get { return base.Manufacturer; } set { base.Manufacturer = value; } }
        [JsonIgnore(Condition = JsonIgnoreCondition.Never)]
        public override string SerialNumber { get { return base.SerialNumber; } set { base.SerialNumber = value; } }

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
