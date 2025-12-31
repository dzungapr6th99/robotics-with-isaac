using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using VDA5050Message.Base;
namespace VDA5050Message
{
    public class Factsheet : VDA5050MessageBase
    {
        public int HeaderId { get; set; }
        public DateTime Timestamp { get; set; }
        public string Version { get; set; }
        public string Manufacturer { get; set; }
        public string SerialNumber { get; set; }

        public TypeSpecification TypeSpecification { get; set; }
        public PhysicalParameters PhysicalParameters { get; set; }
        public ProtocolLimits ProtocolLimits { get; set; }
        public ProtocolFeatures ProtocolFeatures { get; set; }
        public AgvGeometry AgvGeometry { get; set; }
        public LoadSpecification LoadSpecification { get; set; }
        public VehicleConfig? VehicleConfig { get; set; }
    }
}
