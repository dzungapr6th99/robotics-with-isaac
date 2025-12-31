using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace VDA5050Message.Base
{
    public class VehicleConfig
    {
        public List<VehicleVersion>? Versions { get; set; }
        public NetworkConfig? Network { get; set; }
    }

    public class VehicleVersion
    {
        public string Key { get; set; }
        public string Value { get; set; }
    }

    public class NetworkConfig
    {
        public List<string>? DnsServers { get; set; }
        public string? LocalIpAddress { get; set; }
        public List<string>? NtpServers { get; set; }
        public string? Netmask { get; set; }
        public string? DefaultGateway { get; set; }
    }
}
