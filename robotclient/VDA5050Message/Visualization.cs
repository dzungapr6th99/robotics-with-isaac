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
    public class Visualization : VDA5050MessageBase
    {
        private static int _headerId = 0;
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

        public AgvPosition? AgvPosition { get; set; }

        public Velocity? Velocity { get; set; }

        public override void CreateWrapper()
        {
            // Visualization is a "get-only" message (robot sends), so wrapper is created/owned in C++.
        }

        public override void GetDataWrapper(IntPtr prt)
        {
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
