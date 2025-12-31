using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace VDA5050Message.Base
{
    public class Load: VDA5050MessageBase
    {

        public string? LoadId { get; set; }
        public string? LoadType { get; set; }
        public string? LoadPosition { get; set; }
        public BoundingBoxReference? BoundingBoxReference { get; set; }
        public LoadDimensions? LoadDimensions { get; set; }
        public double? Weight { get; set; }

        public override void CreateWrapper() { }
        public override void GetDataWrapper(IntPtr prt) { }
    }

    public class BoundingBoxReference: VDA5050MessageBase
    {
        public double X { get; set; }
        public double Y { get; set; }
        public double Z { get; set; }
        public double? Theta { get; set; }

        public override void CreateWrapper() { }
        public override void GetDataWrapper(IntPtr prt) { }
    }

    public class LoadDimensions: VDA5050MessageBase
    {
        public double Length { get; set; }
        public double Width { get; set; }
        public double? Height { get; set; }

        public override void CreateWrapper() { }
        public override void GetDataWrapper(IntPtr prt) { }
    }
}
