using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Xml.Serialization;

namespace Entity
{
    public class VDA5050Configuration
    {
        [XmlAttribute("interfaceName")]
        public string InterfaceName { get; set; } = string.Empty;
        [XmlAttribute("majorVersion")]
        public string MajorVersion { get; set; } = string.Empty;
        [XmlAttribute("manufacturer")]
        public string Manufacturer { get; set; } = string.Empty;
        [XmlAttribute("serialNumber")]
        public string SerialNumber { get; set; } = string.Empty;
        [XmlAttribute("ipHost")]
        public string IP { get; set; } = string.Empty;
        [XmlAttribute("portHost")]
        public int Port { get; set; }
    }
}
