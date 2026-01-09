using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Text.Json.Serialization;
using System.Threading.Tasks;

namespace VDA5050Message.Base
{
    public class Info: VDA5050MessageBase
    {
        private const string Lib = "libVDAWrapper.so";

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr Info_GetInfoType(IntPtr info);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr Info_GetInfoDescription(IntPtr info);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr Info_GetInfoLevel(IntPtr info);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern int Info_GetInfoReferencesCount(IntPtr info);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr Info_GetInfoReferenceAt(IntPtr info, int index);

        public string InfoType { get; set; }
        public List<InfoReference>? InfoReferences { get; set; }
        public string? InfoDescription { get; set; }

        [JsonConverter(typeof(JsonStringEnumConverter))]
        public InfoLevel InfoLevel { get; set; }

        public override void CreateWrapper() { }
        public override void GetDataWrapper(IntPtr prt)
        {
            InfoType = VDA5050MessageBase.PtrToString(Info_GetInfoType(prt)) ?? "";
            InfoDescription = VDA5050MessageBase.PtrToString(Info_GetInfoDescription(prt));

            var levelText = VDA5050MessageBase.PtrToString(Info_GetInfoLevel(prt));
            if (!string.IsNullOrWhiteSpace(levelText))
            {
                levelText = levelText.Replace("_", "", StringComparison.Ordinal);
                if (Enum.TryParse<InfoLevel>(levelText, true, out var parsedLevel))
                {
                    InfoLevel = parsedLevel;
                }
            }

            InfoReferences ??= new List<InfoReference>();
            InfoReferences.Clear();
            var referenceCount = Info_GetInfoReferencesCount(prt);
            for (var i = 0; i < referenceCount; i++)
            {
                var referencePtr = Info_GetInfoReferenceAt(prt, i);
                if (referencePtr == IntPtr.Zero)
                {
                    continue;
                }
                var reference = new InfoReference();
                reference.GetDataWrapper(referencePtr);
                InfoReferences.Add(reference);
            }
        }
    }

    public class InfoReference: VDA5050MessageBase
    {
        private const string Lib = "libVDAWrapper.so";

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr InfoReference_GetReferenceKey(IntPtr reference);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr InfoReference_GetReferenceValue(IntPtr reference);

        public string ReferenceKey { get; set; }
        public string ReferenceValue { get; set; }

        public override void CreateWrapper() { }
        public override void GetDataWrapper(IntPtr prt)
        {
            ReferenceKey = VDA5050MessageBase.PtrToString(InfoReference_GetReferenceKey(prt)) ?? "";
            ReferenceValue = VDA5050MessageBase.PtrToString(InfoReference_GetReferenceValue(prt)) ?? "";
        }
    }

    public enum InfoLevel
    {
        INFO,
        DEBUG
    }
}
