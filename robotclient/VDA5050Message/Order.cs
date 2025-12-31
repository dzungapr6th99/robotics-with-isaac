
using VDA5050Message.Base;
using System.Runtime.InteropServices;

namespace VDA5050Message
{
    public class Order : VDA5050MessageBase
    {

        private const string Lib = "libVDAWrapper.so";
        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr Order_Create();

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void Order_Destroy(IntPtr orderWrapper);
        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void Order_SetHeaderId(IntPtr orderWrapper, int headerId);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        internal static extern void Order_SetTimeStamp(IntPtr orderWrapper, string? timestamp);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        internal static extern void Order_SetVersion(IntPtr orderWrapper, string? version);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        internal static extern void Order_SetManufacture(IntPtr orderWrapper, string? manufacturer);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        internal static extern void Order_SetSerialNumber(IntPtr orderWrapper, string? serialNumber);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        internal static extern void Order_SetOrderId(IntPtr orderWrapper, string? orderId);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void Order_SetOrderUpdateId(IntPtr orderWrapper, uint orderUpdateId);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        internal static extern void Order_SetZoneSetId(IntPtr orderWrapper, string? zoneSetId);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void Order_ClearNodes(IntPtr orderWrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void Order_AddNode(IntPtr orderWrapper, IntPtr node);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void Order_ClearEdges(IntPtr orderWrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void Order_AddEdge(IntPtr orderWrapper, IntPtr edge);

        public int HeaderId { get; set; }

        public DateTime Timestamp { get; set; }

        public string Version { get; set; }

        public string Manufacturer { get; set; }

        public string SerialNumber { get; set; }

        public string OrderId { get; set; }
        public int OrderUpdateId { get; set; }

        public string? ZoneSetId { get; set; }

        public List<Node> Nodes { get; set; } = new();

        public List<Edge> Edges { get; set; } = new();

        public override void CreateWrapper()
        {
            if (_wrapperPtr.HasValue)
            {
                Order_Destroy(_wrapperPtr.Value);
                _wrapperPtr = null;
            }

            var prt = Order_Create();
            _wrapperPtr = prt;

            Order_SetHeaderId(prt, HeaderId);
            Order_SetTimeStamp(prt, Timestamp.ToString("o"));
            Order_SetVersion(prt, Version);
            Order_SetManufacture(prt, Manufacturer);
            Order_SetSerialNumber(prt, SerialNumber);
            Order_SetOrderId(prt, OrderId);
            Order_SetOrderUpdateId(prt, unchecked((uint)OrderUpdateId));
            Order_SetZoneSetId(prt, ZoneSetId);

            Order_ClearNodes(prt);
            foreach (var node in Nodes)
            {
                node.CreateWrapper();
                if (node._wrapperPtr.HasValue)
                {
                    Order_AddNode(prt, node._wrapperPtr.Value);
                }
            }

            Order_ClearEdges(prt);
            foreach (var edge in Edges)
            {
                edge.CreateWrapper();
                if (edge._wrapperPtr.HasValue)
                {
                    Order_AddEdge(prt, edge._wrapperPtr.Value);
                }
            }
        }

        public override void GetDataWrapper(IntPtr prt)
        {
            // Order is a "set-only" message (robot receives), so C# does not read it from a C++ wrapper.
        }

        ~Order()
        {
            if (_wrapperPtr.HasValue)
            {
                Order_Destroy(_wrapperPtr.Value);
                _wrapperPtr = null;
            }
        }
    }

}
