using System;
using System.Collections.Generic;
using System.Diagnostics.Contracts;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Entity.Api
{
    public class MqttSettingRequest
    {
        public string? Ip {  get; set; }
        public string? Port { get; set; }
        public string? Version { get; set; }
    }

}
