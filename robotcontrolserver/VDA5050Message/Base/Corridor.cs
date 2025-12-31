using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace VDA5050Message.Base
{
    public class Corridor
    {
        public double LeftWidth { get; set; }

        public double RightWidth { get; set; }

        public CorridorRefPoint? CorridorRefPoint { get; set; }
    }

    public enum CorridorRefPoint
    {
        KINEMATICCENTER,
        CONTOUR
    }
}
