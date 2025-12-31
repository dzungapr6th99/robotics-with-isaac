using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace VDA5050Message.Base
{
    public class BatteryState : VDA5050MessageBase
    {
        private const string Lib = "libVDAWrapper.so";

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern double BatteryState_GetBatteryCharge(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern double BatteryState_GetBatteryVoltage(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern sbyte BatteryState_GetBatteryHealth(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern bool BatteryState_GetCharging(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern uint BatteryState_GetReach(IntPtr wrapper);

        public double BatteryCharge { get; set; }
        public double? BatteryVoltage { get; set; }
        public double? BatteryHealth { get; set; }
        public bool Charging { get; set; }
        public double? Reach { get; set; }

        public override void CreateWrapper()
        {
            // BatteryState is a nested "get-only" structure (read from State wrapper).
        }

        public override void GetDataWrapper(IntPtr prt)
        {
            BatteryCharge = BatteryState_GetBatteryCharge(prt);
            BatteryVoltage = BatteryState_GetBatteryVoltage(prt);
            BatteryHealth = BatteryState_GetBatteryHealth(prt);
            Charging = BatteryState_GetCharging(prt);
            Reach = BatteryState_GetReach(prt);
        }
    }
}

