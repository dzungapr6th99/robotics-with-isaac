using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace VDA5050Message.Base
{
    public class Velocity : VDA5050MessageBase
    {
        private const string Lib = "libVDAWrapper.so";

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern double Velocity_GetVx(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern double Velocity_GetVy(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern double Velocity_GetOmega(IntPtr wrapper);

        public double? Vx { get; set; }
        public double? Vy { get; set; }
        public double? Omega { get; set; }

        public override void CreateWrapper()
        {
            // Velocity is a nested "get-only" structure (read from State/Visualization wrapper).
        }

        public override void GetDataWrapper(IntPtr prt)
        {
            Vx = Velocity_GetVx(prt);
            Vy = Velocity_GetVy(prt);
            Omega = Velocity_GetOmega(prt);
        }
    }
}

