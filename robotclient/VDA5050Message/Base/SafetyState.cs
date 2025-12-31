using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Text.Json.Serialization;
using System.Threading.Tasks;

namespace VDA5050Message.Base
{
    public class SafetyState: VDA5050MessageBase
    {
        private const string Lib = "libVDAWrapper.so";

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr SafetyState_GetEStop(IntPtr wrapper);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern bool SafetyState_GetFieldViolation(IntPtr wrapper);

        [JsonConverter(typeof(JsonStringEnumConverter))]
        public EStop EStop { get; set; }

        public bool FieldViolation { get; set; }

        public override void CreateWrapper()
        {
            // SafetyState is a nested "get-only" structure (read from State wrapper).
        }

        public override void GetDataWrapper(IntPtr prt)
        {
            var eStop = VDA5050MessageBase.PtrToString(SafetyState_GetEStop(prt));
            if (Enum.TryParse<EStop>(eStop, true, out var parsedEStop))
            {
                EStop = parsedEStop;
            }
            FieldViolation = SafetyState_GetFieldViolation(prt);
        }
    }

    public enum EStop
    {
        AUTOACK,
        MANUAL,
        REMOTE,
        NONE
    }
}
