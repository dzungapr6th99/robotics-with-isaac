using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace VDA5050Message.Base
{

    public class ControlPoint: VDA5050MessageBase
    {
        private const string Lib = "libVDAWrapper.so";

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr ControlPoint_Create();

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void ControlPoint_Destroy(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void ControlPoint_SetX(IntPtr wrapper, double x);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void ControlPoint_SetY(IntPtr wrapper, double y);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void ControlPoint_SetWeight(IntPtr wrapper, double weight);

        public double X { get; set; }

        public double Y { get; set; }

        public double? Weight { get; set; }

        public override void CreateWrapper()
        {
            if (_wrapperPtr.HasValue)
            {
                ControlPoint_Destroy(_wrapperPtr.Value);
                _wrapperPtr = null;
            }

            var prt = ControlPoint_Create();
            _wrapperPtr = prt;

            ControlPoint_SetX(prt, X);
            ControlPoint_SetY(prt, Y);
            if (Weight.HasValue)
            {
                ControlPoint_SetWeight(prt, Weight.Value);
            }
        }

        public override void GetDataWrapper(IntPtr prt)
        {
            // ControlPoint is used only as a nested "set-only" structure inside Trajectory.
        }

        ~ControlPoint()
        {
            if (_wrapperPtr.HasValue)
            {
                ControlPoint_Destroy(_wrapperPtr.Value);
                _wrapperPtr = null;
            }
        }
    }

}
