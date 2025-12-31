using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;
namespace VDA5050Message.Base
{

    public class Edge : VDA5050MessageBase
    {
        private const string Lib = "libVDAWrapper.so";
        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr Edge_Create();

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void Edge_Destroy(IntPtr edgeWrapper);
        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        internal static extern void Edge_SetEdgeId(IntPtr edgeWrapper, string? edgeId);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void Edge_SetSequenceId(IntPtr edgeWrapper, uint sequenceId);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        internal static extern void Edge_SetDescription(IntPtr edgeWrapper, string? edgeDescription);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void Edge_SetRelease(IntPtr edgeWrapper, bool released);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        internal static extern void Edge_SetStartNodeId(IntPtr edgeWrapper, string? startNodeId);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        internal static extern void Edge_SetEndNodeId(IntPtr edgeWrapper, string? endNodeId);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void Edge_SetMaxSpeed(IntPtr edgeWrapper, double maxSpeed);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void Edge_SetMaxHeight(IntPtr edgeWrapper, double maxHeight);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void Edge_SetMinHeight(IntPtr edgeWrapper, double minHeight);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void Edge_SetOrentation(IntPtr edgeWrapper, double orientation);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        internal static extern void Edge_SetOrientiationType(IntPtr edgeWrapper, string? orientationType);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        internal static extern void Edge_SetDirection(IntPtr edgeWrapper, string? direction);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void Edge_SetRotationAllowed(IntPtr edgeWrapper, bool rotationAllowed);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void Edge_SetMaxRotationSpeed(IntPtr edgeWrapper, double maxRotationSpeed);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void Edge_SetLength(IntPtr edgeWrapper, double length);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void Edge_SetTrajectory(IntPtr edgeWrapper, IntPtr trajectory);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void Edge_ClearActions(IntPtr edgeWrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void Edge_AddAction(IntPtr edgeWrapper, IntPtr action);

        public string EdgeId { get; set; }

        public int SequenceId { get; set; }

        public string? EdgeDescription { get; set; }

        public bool Released { get; set; }

        public string StartNodeId { get; set; }

        public string EndNodeId { get; set; }

        public double? MaxSpeed { get; set; }

        public double? MaxHeight { get; set; }
        public double? MinHeight { get; set; }

        public double? Orientation { get; set; }

        public string? OrientationType { get; set; }

        public string? Direction { get; set; }

        public bool? RotationAllowed { get; set; }

        public double? MaxRotationSpeed { get; set; }

        public double? Length { get; set; }

        public Trajectory? Trajectory { get; set; }

        public Corridor? Corridor { get; set; }

        public List<Action> Actions { get; set; } = new();

        public override void CreateWrapper()
        {
            if (_wrapperPtr.HasValue)
            {
                Edge_Destroy(_wrapperPtr.Value);
                _wrapperPtr = null;
            }

            var prt = Edge_Create();
            _wrapperPtr = prt;

            Edge_SetEdgeId(prt, EdgeId);
            Edge_SetSequenceId(prt, unchecked((uint)SequenceId));
            Edge_SetDescription(prt, EdgeDescription);
            Edge_SetRelease(prt, Released);
            Edge_SetStartNodeId(prt, StartNodeId);
            Edge_SetEndNodeId(prt, EndNodeId);

            if (MaxSpeed.HasValue) Edge_SetMaxSpeed(prt, MaxSpeed.Value);
            if (MaxHeight.HasValue) Edge_SetMaxHeight(prt, MaxHeight.Value);
            if (MinHeight.HasValue) Edge_SetMinHeight(prt, MinHeight.Value);
            if (Orientation.HasValue) Edge_SetOrentation(prt, Orientation.Value);
            if (!string.IsNullOrWhiteSpace(OrientationType)) Edge_SetOrientiationType(prt, OrientationType);
            if (!string.IsNullOrWhiteSpace(Direction)) Edge_SetDirection(prt, Direction);
            if (RotationAllowed.HasValue) Edge_SetRotationAllowed(prt, RotationAllowed.Value);
            if (MaxRotationSpeed.HasValue) Edge_SetMaxRotationSpeed(prt, MaxRotationSpeed.Value);
            if (Length.HasValue) Edge_SetLength(prt, Length.Value);

            if (Trajectory != null)
            {
                Trajectory.CreateWrapper();
                if (Trajectory._wrapperPtr.HasValue)
                {
                    Edge_SetTrajectory(prt, Trajectory._wrapperPtr.Value);
                }
            }

            Edge_ClearActions(prt);
            foreach (var action in Actions)
            {
                action.CreateWrapper();
                if (action._wrapperPtr.HasValue)
                {
                    Edge_AddAction(prt, action._wrapperPtr.Value);
                }
            }
        }

        public override void GetDataWrapper(IntPtr prt)
        {
            // Edge is used only as a nested "set-only" structure inside Order.
        }

        ~Edge()
        {
            if (_wrapperPtr.HasValue)
            {
                Edge_Destroy(_wrapperPtr.Value);
                _wrapperPtr = null;
            }
        }
    }

    public class EdgeState : VDA5050MessageBase
    {
        private const string Lib = "libVDAWrapper.so";

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr EdgeState_GetEdgeId(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern int EdgeState_GetSequenceId(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr EdgeState_GetEdgeDescription(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern bool EdgeState_GetReleased(IntPtr wrapper);

        public string EdgeId { get; set; }
        public int SequenceId { get; set; }
        public string? EdgeDescription { get; set; }
        public bool Released { get; set; }
        public Trajectory? Trajectory { get; set; }

        public override void CreateWrapper()
        {
            // EdgeState is a "get-only" structure (robot sends State), so wrapper is created/owned in C++.
        }

        public override void GetDataWrapper(IntPtr prt)
        {
            EdgeId = VDA5050MessageBase.PtrToString(EdgeState_GetEdgeId(prt)) ?? "";
            SequenceId = EdgeState_GetSequenceId(prt);
            EdgeDescription = VDA5050MessageBase.PtrToString(EdgeState_GetEdgeDescription(prt));
            Released = EdgeState_GetReleased(prt);
        }
    }

}
