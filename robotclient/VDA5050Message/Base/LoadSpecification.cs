using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace VDA5050Message.Base
{
    public class LoadSpecification: VDA5050MessageBase
    {
        public List<string>? LoadPositions { get; set; }
        public List<LoadSet>? LoadSets { get; set; }

        public override void CreateWrapper() { }
        public override void GetDataWrapper(IntPtr prt) { }
    }

    public class LoadSet: VDA5050MessageBase
    {
        public string SetName { get; set; }
        public string LoadType { get; set; }
        public List<string>? LoadPositions { get; set; }

        public BoundingBoxReference? BoundingBoxReference { get; set; }
        public LoadDimensions? LoadDimensions { get; set; }

        public double? MaxWeight { get; set; }
        public double? MinLoadhandlingHeight { get; set; }
        public double? MaxLoadhandlingHeight { get; set; }
        public double? MinLoadhandlingDepth { get; set; }
        public double? MaxLoadhandlingDepth { get; set; }
        public double? MinLoadhandlingTilt { get; set; }
        public double? MaxLoadhandlingTilt { get; set; }
        public double? AgvSpeedLimit { get; set; }
        public double? AgvAccelerationLimit { get; set; }
        public double? AgvDecelerationLimit { get; set; }
        public double? PickTime { get; set; }
        public double? DropTime { get; set; }
        public string? Description { get; set; }

        public override void CreateWrapper() { }
        public override void GetDataWrapper(IntPtr prt) { }
    }

}
