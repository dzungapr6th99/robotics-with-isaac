using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;
using VDA5050Message.Base;

namespace VDA5050Message
{
    public class Visualization :VDA5050MessageBase
    {
        private const string Lib = "libVDAWrapper.so";

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern int Visualization_GetHeaderId(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr Visualization_GetTimestamp(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr Visualization_GetVersion(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr Visualization_GetManufacturer(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr Visualization_GetSerialNumber(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr Visualization_GetAGVPosition(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr Visualization_GetVelocity(IntPtr wrapper);

        public int? HeaderId { get; set; }

        public DateTime? Timestamp { get; set; }

        public string? Version { get; set; }

        public string? Manufacturer { get; set; }

        public string? SerialNumber { get; set; }

        public AgvPosition? AgvPosition { get; set; }

        public Velocity? Velocity { get; set; }

        public override void CreateWrapper()
        {
            // Visualization is a "get-only" message (robot sends), so wrapper is created/owned in C++.
        }

        public override void GetDataWrapper(IntPtr prt)
        {
            HeaderId = Visualization_GetHeaderId(prt);
            var timestamp = VDA5050MessageBase.PtrToString(Visualization_GetTimestamp(prt));
            if (DateTime.TryParse(timestamp, out var parsedTimestamp))
            {
                Timestamp = parsedTimestamp;
            }
            Version = VDA5050MessageBase.PtrToString(Visualization_GetVersion(prt));
            Manufacturer = VDA5050MessageBase.PtrToString(Visualization_GetManufacturer(prt));
            SerialNumber = VDA5050MessageBase.PtrToString(Visualization_GetSerialNumber(prt));

            var agvPositionPtr = Visualization_GetAGVPosition(prt);
            if (agvPositionPtr != IntPtr.Zero)
            {
                AgvPosition ??= new AgvPosition();
                AgvPosition.GetDataWrapper(agvPositionPtr);
            }

            var velocityPtr = Visualization_GetVelocity(prt);
            if (velocityPtr != IntPtr.Zero)
            {
                Velocity ??= new Velocity();
                Velocity.GetDataWrapper(velocityPtr);
            }
        }
    }
}
