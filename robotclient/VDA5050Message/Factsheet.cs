using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;
using VDA5050Message.Base;
namespace VDA5050Message
{
    public class Factsheet : VDA5050MessageBase
    {
        private const string Lib = "libVDAWrapper.so";

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern int Factsheet_GetHeaderId(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr Factsheet_GetTimestamp(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr Factsheet_GetVersion(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr Factsheet_GetManufacturer(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr Factsheet_GetSerialNumber(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr Factsheet_GetTypeSpecification(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr Factsheet_GetPhysicalParameters(IntPtr wrapper);

        public int HeaderId { get; set; }
        public DateTime Timestamp { get; set; }
        public string Version { get; set; }
        public string Manufacturer { get; set; }
        public string SerialNumber { get; set; }

        public TypeSpecification TypeSpecification { get; set; }
        public PhysicalParameters PhysicalParameters { get; set; }
        public ProtocolLimits ProtocolLimits { get; set; }
        public ProtocolFeatures ProtocolFeatures { get; set; }
        public AgvGeometry AgvGeometry { get; set; }
        public LoadSpecification LoadSpecification { get; set; }
        public VehicleConfig? VehicleConfig { get; set; }

        public override void CreateWrapper()
        {
            // Factsheet is a "get-only" message (robot sends), so wrapper is created/owned in C++.
        }

        public override void GetDataWrapper(IntPtr prt)
        {
            HeaderId = Factsheet_GetHeaderId(prt);
            var timestamp = VDA5050MessageBase.PtrToString(Factsheet_GetTimestamp(prt));
            if (DateTime.TryParse(timestamp, out var parsedTimestamp))
            {
                Timestamp = parsedTimestamp;
            }
            Version = VDA5050MessageBase.PtrToString(Factsheet_GetVersion(prt)) ?? "";
            Manufacturer = VDA5050MessageBase.PtrToString(Factsheet_GetManufacturer(prt)) ?? "";
            SerialNumber = VDA5050MessageBase.PtrToString(Factsheet_GetSerialNumber(prt)) ?? "";

            var typeSpecPtr = Factsheet_GetTypeSpecification(prt);
            if (typeSpecPtr != IntPtr.Zero)
            {
                TypeSpecification ??= new TypeSpecification();
                TypeSpecification.GetDataWrapper(typeSpecPtr);
            }

            var physicalParametersPtr = Factsheet_GetPhysicalParameters(prt);
            if (physicalParametersPtr != IntPtr.Zero)
            {
                PhysicalParameters ??= new PhysicalParameters();
                PhysicalParameters.GetDataWrapper(physicalParametersPtr);
            }
        }
    }
}
