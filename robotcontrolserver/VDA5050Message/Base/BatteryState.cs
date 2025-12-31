using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace VDA5050Message.Base
{
    public class BatteryState
    {
        public double BatteryCharge { get; set; }
        public double? BatteryVoltage { get; set; }
        public double? BatteryHealth { get; set; }
        public bool Charging { get; set; }
        public double? Reach { get; set; }
    }
}
