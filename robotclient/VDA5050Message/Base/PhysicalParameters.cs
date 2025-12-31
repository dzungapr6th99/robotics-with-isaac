using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace VDA5050Message.Base
{
    public class PhysicalParameters : VDA5050MessageBase
    {
        private const string Lib = "libVDAWrapper.so";
        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern double PhysicalParameters_GetSpeedMin(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern double PhysicalParameters_GetSpeedMax(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern double PhysicalParameters_GetAccelerationMax(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern double PhysicalParameters_GetDecelerationMax(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern double PhysicalParameters_GetHeightMin(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern double PhysicalParameters_GetHeightMax(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern double PhysicalParameters_GetWidth(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern double PhysicalParameters_GetLength(IntPtr wrapper);
        public double SpeedMin { get; set; }
        public double SpeedMax { get; set; }
        public double AccelerationMax { get; set; }
        public double DecelerationMax { get; set; }
        public double? HeightMin { get; set; }
        public double HeightMax { get; set; }
        public double Width { get; set; }
        public double Length { get; set; }

        public override void CreateWrapper()
        {
            // PhysicalParameters is a nested "get-only" structure (read from Factsheet wrapper).
        }

        public override void GetDataWrapper(IntPtr prt)
        {
            SpeedMin = PhysicalParameters_GetSpeedMin(prt);
            SpeedMax = PhysicalParameters_GetSpeedMax(prt);
            AccelerationMax = PhysicalParameters_GetAccelerationMax(prt);
            DecelerationMax = PhysicalParameters_GetDecelerationMax(prt);
            HeightMin = PhysicalParameters_GetHeightMin(prt);
            HeightMax = PhysicalParameters_GetHeightMax(prt);
            Width = PhysicalParameters_GetWidth(prt);
            Length = PhysicalParameters_GetLength(prt);
        }
    }
}
