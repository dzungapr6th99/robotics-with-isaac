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

        [JsonIgnore(Condition = JsonIgnoreCondition.Never)]
        public override int HeaderId {get { return base.HeaderId; } set { base.HeaderId = value; } }

        [JsonIgnore(Condition = JsonIgnoreCondition.Never)]
        public override DateTime Timestamp {get { return base.Timestamp; } set { base.Timestamp = value; } }

        [JsonIgnore(Condition = JsonIgnoreCondition.Never)]
        public override string Version {get { return base.Version; } set { base.Version = value; } }

        [JsonIgnore(Condition = JsonIgnoreCondition.Never)]
        public override string Manufacturer {get { return base.Manufacturer; } set { base.Manufacturer = value; } }

        [JsonIgnore(Condition = JsonIgnoreCondition.Never)]
        public override string SerialNumber {get { return base.SerialNumber; } set { base.SerialNumber = value; } }

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
