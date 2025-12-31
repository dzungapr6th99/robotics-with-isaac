
using VDA5050Message.Base;

namespace VDA5050Message
{
    public class Order : VDA5050MessageBase
    {
        public int HeaderId { get; set; }

        public DateTime Timestamp { get; set; }

        public string Version { get; set; }

        public string Manufacturer { get; set; }

        public string SerialNumber { get; set; }

        public string OrderId { get; set; }
        public int OrderUpdateId { get; set; }

        public string? ZoneSetId { get; set; }

        public List<Node> Nodes { get; set; } = new();

        public List<Edge> Edges { get; set; } = new();
    }

}
