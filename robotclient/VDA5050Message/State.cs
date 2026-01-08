using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Text.Json.Serialization;
using System.Threading.Tasks;
using VDA5050Message.Base;
namespace VDA5050Message
{
    public class State : VDA5050MessageBase
    {
        private const string Lib = "libVDAWrapper.so";
        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern int AGVState_GetHeaderId(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr AGVState_GetTimestamp(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr AGVState_GetVersion(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr AGVState_GetManufacturer(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr AGVState_GetSerialNumber(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr AGVState_GetOrderId(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern uint AGVState_GetOrderUpdateId(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr AGVState_GetZoneSetId(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr AGVState_GetLastNodeId(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern int AGVState_GetLastNodeSequenceId(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern bool AGVState_GetDriving(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern bool AGVState_GetPaused(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern bool AGVState_GetNewBaseRequested(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern double AGVState_GetDistanceSinceLastNode(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr AGVState_GetOperatingMode(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr AGVState_GetNodeStateAt(IntPtr wrapper, int index);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr AGVState_GetEdgeStateAt(IntPtr wrapper, int index);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr AGVState_GetNodeStates(IntPtr wrapper, out int length);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr AGVState_GetEdgeStates(IntPtr wrapper, out int length);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr AGVState_GetAGVPosition(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr AGVState_GetVelocity(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr AGVState_GetActionStates(IntPtr wrapper, out int length);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr AGVState_GetActionStateAt(IntPtr wrapper, int index);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr AGVState_GetBatteryState(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr AGVState_GetSafetyState(IntPtr wrapper);


        [JsonIgnore(Condition = JsonIgnoreCondition.Never)]
        public override int HeaderId { get { return base.HeaderId; } set { base.HeaderId = value; } }

        [JsonIgnore(Condition = JsonIgnoreCondition.Never)]
        public override DateTime Timestamp { get { return base.Timestamp; } set { base.Timestamp = value; } }

        [JsonIgnore(Condition = JsonIgnoreCondition.Never)]
        public override string Version { get { return base.Version; } set { base.Version = value; } }

        [JsonIgnore(Condition = JsonIgnoreCondition.Never)]
        public override string Manufacturer { get { return base.Manufacturer; } set { base.Manufacturer = value; } }

        [JsonIgnore(Condition = JsonIgnoreCondition.Never)]
        public override string SerialNumber { get { return base.SerialNumber; } set { base.SerialNumber = value; } }
        public List<Map>? Maps { get; set; }

        public string OrderId { get; set; }
        public int OrderUpdateId { get; set; }
        public string? ZoneSetId { get; set; }
        public string LastNodeId { get; set; }
        public int LastNodeSequenceId { get; set; }
        public bool Driving { get; set; }
        public bool? Paused { get; set; }
        public bool? NewBaseRequest { get; set; }
        public double? DistanceSinceLastNode { get; set; }

        [JsonConverter(typeof(JsonStringEnumConverter))]
        public OperatingMode OperatingMode { get; set; }

        public List<NodeState> NodeStates { get; set; }
        public List<EdgeState> EdgeStates { get; set; }

        public AgvPosition? AgvPosition { get; set; }
        public Velocity? Velocity { get; set; }
        public List<Load>? Loads { get; set; }
        public List<ActionState> ActionStates { get; set; }
        public BatteryState BatteryState { get; set; }
        public List<Error> Errors { get; set; }
        public List<Info>? Information { get; set; }
        public SafetyState SafetyState { get; set; }

        public override void CreateWrapper()
        {
            // State is a "get-only" message (robot sends), so wrapper is created/owned in C++.
        }

        public override void GetDataWrapper(IntPtr prt)
        {
            OrderId = VDA5050MessageBase.PtrToString(AGVState_GetOrderId(prt)) ?? "";
            OrderUpdateId = unchecked((int)AGVState_GetOrderUpdateId(prt));
            ZoneSetId = VDA5050MessageBase.PtrToString(AGVState_GetZoneSetId(prt));
            LastNodeId = VDA5050MessageBase.PtrToString(AGVState_GetLastNodeId(prt)) ?? "";
            LastNodeSequenceId = AGVState_GetLastNodeSequenceId(prt);

            Driving = AGVState_GetDriving(prt);
            Paused = AGVState_GetPaused(prt);
            NewBaseRequest = AGVState_GetNewBaseRequested(prt);
            DistanceSinceLastNode = AGVState_GetDistanceSinceLastNode(prt);

            var operatingMode = VDA5050MessageBase.PtrToString(AGVState_GetOperatingMode(prt));
            if (Enum.TryParse<OperatingMode>(operatingMode, true, out var parsedOperatingMode))
            {
                OperatingMode = parsedOperatingMode;
            }

            NodeStates ??= new List<NodeState>();
            NodeStates.Clear();
            AGVState_GetNodeStates(prt, out var nodeCount);
            for (var i = 0; i < nodeCount; i++)
            {
                var nodeStatePtr = AGVState_GetNodeStateAt(prt, i);
                if (nodeStatePtr == IntPtr.Zero)
                {
                    continue;
                }
                var nodeState = new NodeState();
                nodeState.GetDataWrapper(nodeStatePtr);
                NodeStates.Add(nodeState);
            }

            EdgeStates ??= new List<EdgeState>();
            EdgeStates.Clear();
            AGVState_GetEdgeStates(prt, out var edgeCount);
            for (var i = 0; i < edgeCount; i++)
            {
                var edgeStatePtr = AGVState_GetEdgeStateAt(prt, i);
                if (edgeStatePtr == IntPtr.Zero)
                {
                    continue;
                }
                var edgeState = new EdgeState();
                edgeState.GetDataWrapper(edgeStatePtr);
                EdgeStates.Add(edgeState);
            }

            var agvPositionPtr = AGVState_GetAGVPosition(prt);
            if (agvPositionPtr != IntPtr.Zero)
            {
                AgvPosition ??= new AgvPosition();
                AgvPosition.GetDataWrapper(agvPositionPtr);
            }

            var velocityPtr = AGVState_GetVelocity(prt);
            if (velocityPtr != IntPtr.Zero)
            {
                Velocity ??= new Velocity();
                Velocity.GetDataWrapper(velocityPtr);
            }

            ActionStates ??= new List<ActionState>();
            ActionStates.Clear();
            AGVState_GetActionStates(prt, out var actionStateCount);
            for (var i = 0; i < actionStateCount; i++)
            {
                var actionStatePtr = AGVState_GetActionStateAt(prt, i);
                if (actionStatePtr == IntPtr.Zero)
                {
                    continue;
                }
                var actionState = new ActionState();
                actionState.GetDataWrapper(actionStatePtr);
                ActionStates.Add(actionState);
            }

            var batteryStatePtr = AGVState_GetBatteryState(prt);
            if (batteryStatePtr != IntPtr.Zero)
            {
                BatteryState ??= new BatteryState();
                BatteryState.GetDataWrapper(batteryStatePtr);
            }

            var safetyStatePtr = AGVState_GetSafetyState(prt);
            if (safetyStatePtr != IntPtr.Zero)
            {
                SafetyState ??= new SafetyState();
                SafetyState.GetDataWrapper(safetyStatePtr);
            }
        }
    }




}
