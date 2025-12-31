using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace VDA5050Message.Base
{
    public class ProtocolLimits: VDA5050MessageBase
    {
        public MaxStringLens? MaxStringLens { get; set; }
        public MaxArrayLens? MaxArrayLens { get; set; }
        public Timing Timing { get; set; }

        public override void CreateWrapper() { }
        public override void GetDataWrapper(IntPtr prt) { }
    }

    public class MaxStringLens: VDA5050MessageBase
    {
        public int? MsgLen { get; set; }
        public int? TopicSerialLen { get; set; }
        public int? TopicElemLen { get; set; }
        public int? IdLen { get; set; }
        public bool? IdNumericalOnly { get; set; }
        public int? EnumLen { get; set; }
        public int? LoadIdLen { get; set; }

        public override void CreateWrapper() { }
        public override void GetDataWrapper(IntPtr prt) { }
    }

    public class MaxArrayLens: VDA5050MessageBase
    {
        public int? OrderNodes { get; set; }
        public int? OrderEdges { get; set; }
        public int? NodeActions { get; set; }
        public int? EdgeActions { get; set; }
        public int? ActionsActionsParameters { get; set; }
        public int? InstantActions { get; set; }
        public int? TrajectoryKnotVector { get; set; }
        public int? TrajectoryControlPoints { get; set; }
        public int? StateNodeStates { get; set; }
        public int? StateEdgeStates { get; set; }
        public int? StateLoads { get; set; }
        public int? StateActionStates { get; set; }
        public int? StateErrors { get; set; }
        public int? StateInformation { get; set; }
        public int? ErrorErrorReferences { get; set; }
        public int? InformationInfoReferences { get; set; }

        public override void CreateWrapper() { }
        public override void GetDataWrapper(IntPtr prt) { }
    }

    public class Timing: VDA5050MessageBase
    {
        public double MinOrderInterval { get; set; }
        public double MinStateInterval { get; set; }
        public double? DefaultStateInterval { get; set; }
        public double? VisualizationInterval { get; set; }

        public override void CreateWrapper() { }
        public override void GetDataWrapper(IntPtr prt) { }
    }
}
