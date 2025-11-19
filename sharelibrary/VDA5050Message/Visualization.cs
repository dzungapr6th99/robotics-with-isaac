using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using VDA5050Message.Base;

namespace VDA5050Message
{
    public class Visualization :VDA5050MessageBase
    {
        public int? HeaderId { get; set; }

        public DateTime? Timestamp { get; set; }

        public string? Version { get; set; }

        public string? Manufacturer { get; set; }

        public string? SerialNumber { get; set; }

        public AgvPosition? AgvPosition { get; set; }

        public Velocity? Velocity { get; set; }
    }
}
