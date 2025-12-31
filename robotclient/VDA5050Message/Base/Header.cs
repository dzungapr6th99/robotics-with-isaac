using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace VDA5050Message.Base
{
    public class Header: VDA5050MessageBase
    {
        public string? HeaderId { get; set; }
        public string? TimeStamp {  get; set; }
        public string? Version {  get; set; }
        public string? Manufacturer {  get; set; }
        public string? SerialNumber {  get; set; }

        public override void CreateWrapper() { }
        public override void GetDataWrapper(IntPtr prt) { }
    }
}
