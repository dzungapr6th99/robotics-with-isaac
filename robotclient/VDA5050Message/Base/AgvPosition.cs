using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace VDA5050Message.Base
{
    public class AgvPosition : VDA5050MessageBase
    {
        private const string Lib = "libVDAWrapper.so";
        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern double AGVPosition_GetX(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern double AGVPosition_GetY(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern double AGVPosition_GetTheta(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr AGVPosition_GetMapId(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr AGVPosition_GetMapDescription(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern bool AGVPosition_GetPositionInitialized(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern double AGVPosition_GetLocalizationScore(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern double AGVPosition_GetDeviationRange(IntPtr wrapper);

        public double X { get; set; }

        public double Y { get; set; }

        public double Theta { get; set; }

        public string MapId { get; set; }

        public string? MapDescription { get; set; }

        public bool PositionInitialized { get; set; }

        public double? LocalizationScore { get; set; }

        public double? DeviationRange { get; set; }

        public override void CreateWrapper()
        {
            // AgvPosition is a nested "get-only" structure (read from State/Visualization wrapper).
        }

        public override void GetDataWrapper(IntPtr prt)
        {
            X = AGVPosition_GetX(prt);
            Y = AGVPosition_GetY(prt);
            Theta = AGVPosition_GetTheta(prt);
            MapId = VDA5050MessageBase.PtrToString(AGVPosition_GetMapId(prt)) ?? "";
            MapDescription = VDA5050MessageBase.PtrToString(AGVPosition_GetMapDescription(prt));
            PositionInitialized = AGVPosition_GetPositionInitialized(prt);
            LocalizationScore = AGVPosition_GetLocalizationScore(prt);
            DeviationRange = AGVPosition_GetDeviationRange(prt);
        }
    }
}
