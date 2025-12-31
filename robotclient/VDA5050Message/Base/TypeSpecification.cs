using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Text.Json.Serialization;
using System.Threading.Tasks;

namespace VDA5050Message.Base
{
    public class TypeSpecification : VDA5050MessageBase
    {
        private const string Lib = "libVDAWrapper.so";

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr TypeSpecification_GetSeriesName(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr TypeSpecification_GetSeriesDescription(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr TypeSpecification_GetAgvKinematics(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr TypeSpecification_GetAgvClass(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern double TypeSpecification_GetMaxLoadMass(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr TypeSpecification_GetLocalizationTypes(IntPtr wrapper, out int length);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr TypeSpecification_GetNavigationTypes(IntPtr wrapper, out int length);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr TypeSpecification_GetStringData(IntPtr data, int index);

        public string SeriesName { get; set; }
        public string? SeriesDescription { get; set; }

        [JsonConverter(typeof(JsonStringEnumConverter))]
        public AgvKinematic AgvKinematic { get; set; }

        [JsonConverter(typeof(JsonStringEnumConverter))]
        public AgvClass AgvClass { get; set; }

        public double MaxLoadMass { get; set; }
        public List<LocalizationType> LocalizationTypes { get; set; }
        public List<NavigationType> NavigationTypes { get; set; }

        public override void CreateWrapper()
        {
            // TypeSpecification is a nested "get-only" structure (read from Factsheet wrapper).
        }

        public override void GetDataWrapper(IntPtr prt)
        {
            SeriesName = VDA5050MessageBase.PtrToString(TypeSpecification_GetSeriesName(prt)) ?? "";
            SeriesDescription = VDA5050MessageBase.PtrToString(TypeSpecification_GetSeriesDescription(prt));

            var kinematics = VDA5050MessageBase.PtrToString(TypeSpecification_GetAgvKinematics(prt));
            if (Enum.TryParse<AgvKinematic>(kinematics, true, out var parsedAgvKinematic))
            {
                AgvKinematic = parsedAgvKinematic;
            }

            var agvClass = VDA5050MessageBase.PtrToString(TypeSpecification_GetAgvClass(prt));
            if (Enum.TryParse<AgvClass>(agvClass, true, out var parsedAgvClass))
            {
                AgvClass = parsedAgvClass;
            }

            MaxLoadMass = TypeSpecification_GetMaxLoadMass(prt);

            LocalizationTypes = new List<LocalizationType>();
            var localizationPtr = TypeSpecification_GetLocalizationTypes(prt, out var localizationCount);
            for (var i = 0; i < localizationCount; i++)
            {
                var s = VDA5050MessageBase.PtrToString(TypeSpecification_GetStringData(localizationPtr, i));
                if (Enum.TryParse<LocalizationType>(s, true, out var parsed))
                {
                    LocalizationTypes.Add(parsed);
                }
            }

            NavigationTypes = new List<NavigationType>();
            var navigationPtr = TypeSpecification_GetNavigationTypes(prt, out var navigationCount);
            for (var i = 0; i < navigationCount; i++)
            {
                var s = VDA5050MessageBase.PtrToString(TypeSpecification_GetStringData(navigationPtr, i));
                if (Enum.TryParse<NavigationType>(s, true, out var parsed))
                {
                    NavigationTypes.Add(parsed);
                }
            }
        }
    }

    public enum AgvKinematic
    {
        DIFF,
        OMNI,
        THREEWHEEL
    }

    public enum AgvClass
    {
        FORKLIFT,
        CONVEYOR,
        TUGGER,
        CARRIER
    }

    public enum LocalizationType
    {
        NATURAL,
        REFLECTOR,
        RFID,
        DMC,
        SPOT,
        GRID
    }

    public enum NavigationType
    {
        PHYSICAL_LINE_GUIDED,
        VIRTUAL_LINE_GUIDED,
        AUTONOMOUS
    }
}

