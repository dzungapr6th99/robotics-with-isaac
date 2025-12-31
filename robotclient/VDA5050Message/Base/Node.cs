using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Runtime.InteropServices;
namespace VDA5050Message.Base
{
    public class Node : VDA5050MessageBase
    {
        private const string Lib = "libVDAWrapper.so";
        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr Node_Create();

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void Node_Destroy(IntPtr nodeWrapper);
        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        internal static extern void Node_SetNodeId(IntPtr nodeWrapper, string? nodeId);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void Node_SetSequenceId(IntPtr nodeWrapper, uint sequenceId);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        internal static extern void Node_SetNodeDescription(IntPtr nodeWrapper, string? nodeDescription);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void Node_SetReleased(IntPtr nodeWrapper, bool released);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void Node_SetNodePosition(IntPtr nodeWrapper, IntPtr nodePosition);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void Node_ClearActions(IntPtr nodeWrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void Node_AddAction(IntPtr nodeWrapper, IntPtr action);

        public string NodeId { get; set; }

        public int SequenceId { get; set; }

        public string? NodeDescription { get; set; }

        public bool Released { get; set; }

        public NodePosition? NodePosition { get; set; }

        public List<Action> Actions { get; set; } = new();

        public override void CreateWrapper()
        {
            if (_wrapperPtr.HasValue)
            {
                Node_Destroy(_wrapperPtr.Value);
                _wrapperPtr = null;
            }

            var prt = Node_Create();
            _wrapperPtr = prt;

            Node_SetNodeId(prt, NodeId);
            Node_SetSequenceId(prt, unchecked((uint)SequenceId));
            Node_SetNodeDescription(prt, NodeDescription);
            Node_SetReleased(prt, Released);

            if (NodePosition != null)
            {
                NodePosition.CreateWrapper();
                if (NodePosition._wrapperPtr.HasValue)
                {
                    Node_SetNodePosition(prt, NodePosition._wrapperPtr.Value);
                }
            }

            Node_ClearActions(prt);
            foreach (var action in Actions)
            {
                action.CreateWrapper();
                if (action._wrapperPtr.HasValue)
                {
                    Node_AddAction(prt, action._wrapperPtr.Value);
                }
            }
        }

        public override void GetDataWrapper(IntPtr prt)
        {
            // Node is used only as a nested "set-only" structure inside Order.
        }

        ~Node()
        {
            if (_wrapperPtr.HasValue)
            {
                Node_Destroy(_wrapperPtr.Value);
                _wrapperPtr = null;
            }
        }
    }


    public class NodePosition : VDA5050MessageBase
    {
        private const string Lib = "libVDAWrapper.so";

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr NodePosition_Create();

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void NodePosition_Destroy(IntPtr nodePositionWrapper);



        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void NodePosition_SetX(IntPtr wrapper, double x);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void NodePosition_SetY(IntPtr wrapper, double y);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void NodePosition_SetTheta(IntPtr wrapper, double theta);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void NodePosition_SetAllowedDeviationXY(IntPtr wrapper, float allowedDeviationXY);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void NodePosition_SetAllowedDeviationTheta(IntPtr wrapper, float allowedDeviationTheta);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        internal static extern void NodePosition_SetMapId(IntPtr wrapper, string? mapId);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        internal static extern void NodePosition_SetMapDescription(IntPtr wrapper, string? mapDescription);

        public double X { get; set; }

        public double Y { get; set; }

        public double? Theta { get; set; }

        public double? AllowedDeviationXY { get; set; }

        public double? AllowedDeviationTheta { get; set; }

        public string MapId { get; set; }

        public string? MapDescription { get; set; }

        public override void CreateWrapper()
        {
            if (_wrapperPtr.HasValue)
            {
                NodePosition_Destroy(_wrapperPtr.Value);
                _wrapperPtr = null;
            }

            var prt = NodePosition_Create();
            _wrapperPtr = prt;

            NodePosition_SetX(prt, X);
            NodePosition_SetY(prt, Y);
            if (Theta.HasValue)
            {
                NodePosition_SetTheta(prt, Theta.Value);
            }
            if (AllowedDeviationXY.HasValue)
            {
                NodePosition_SetAllowedDeviationXY(prt, (float)AllowedDeviationXY.Value);
            }
            if (AllowedDeviationTheta.HasValue)
            {
                NodePosition_SetAllowedDeviationTheta(prt, (float)AllowedDeviationTheta.Value);
            }
            NodePosition_SetMapId(prt, MapId);
            NodePosition_SetMapDescription(prt, MapDescription);
        }

        public override void GetDataWrapper(IntPtr prt)
        {
            // NodePosition is used only as a nested "set-only" structure inside Node.
        }

        ~NodePosition()
        {
            if (_wrapperPtr.HasValue)
            {
                NodePosition_Destroy(_wrapperPtr.Value);
                _wrapperPtr = null;
            }
        }
    }


    public class NodeState : VDA5050MessageBase
    {
        private const string Lib = "libVDAWrapper.so";
        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr NodeState_GetNodeId(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern int NodeState_GetSequenceId(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr NodeState_GetNodeDescription(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern bool NodeState_GetReleased(IntPtr wrapper);
        public string NodeId { get; set; }
        public int SequenceId { get; set; }
        public string? NodeDescription { get; set; }
        public bool Released { get; set; }
        public Node? NodePosition { get; set; }

        public override void CreateWrapper()
        {
            // NodeState is a "get-only" structure (robot sends State), so wrapper is created/owned in C++.
        }

        public override void GetDataWrapper(IntPtr prt)
        {
            NodeId = VDA5050MessageBase.PtrToString(NodeState_GetNodeId(prt)) ?? "";
            SequenceId = NodeState_GetSequenceId(prt);
            NodeDescription = VDA5050MessageBase.PtrToString(NodeState_GetNodeDescription(prt));
            Released = NodeState_GetReleased(prt);
        }
    }

}
