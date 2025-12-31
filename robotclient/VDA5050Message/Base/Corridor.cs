using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace VDA5050Message.Base
{
    public class Corridor: VDA5050MessageBase
    {
        public double LeftWidth { get; set; }

        public double RightWidth { get; set; }

        public CorridorRefPoint? CorridorRefPoint { get; set; }

        public override void CreateWrapper() { }
        public override void GetDataWrapper(IntPtr prt) { }
    }

    public enum CorridorRefPoint
    {
        KINEMATICCENTER,
        CONTOUR
    }
}
