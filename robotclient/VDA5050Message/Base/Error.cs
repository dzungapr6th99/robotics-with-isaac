using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Text.Json.Serialization;
using System.Threading.Tasks;

namespace VDA5050Message.Base
{
    public class Error: VDA5050MessageBase
    {
        private const string Lib = "libVDAWrapper.so";

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr Error_GetErrorType(IntPtr error);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr Error_GetErrorDescription(IntPtr error);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr Error_GetErrorLevel(IntPtr error);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern int Error_GetErrorReferencesCount(IntPtr error);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr Error_GetErrorReferenceAt(IntPtr error, int index);

        public string ErrorType { get; set; }
        public List<ErrorReference>? ErrorReferences { get; set; }
        public string? ErrorDescription { get; set; }
        public string? ErrorHint { get; set; }

        [JsonConverter(typeof(JsonStringEnumConverter))]
        public ErrorLevel ErrorLevel { get; set; }

        public override void CreateWrapper()
        {
            // Error is a nested "get-only" structure (read from State wrapper).
        }

        public override void GetDataWrapper(IntPtr prt)
        {
            ErrorType = VDA5050MessageBase.PtrToString(Error_GetErrorType(prt)) ?? "";
            ErrorDescription = VDA5050MessageBase.PtrToString(Error_GetErrorDescription(prt));
            ErrorHint = null;

            var level = VDA5050MessageBase.PtrToString(Error_GetErrorLevel(prt));
            if (Enum.TryParse<ErrorLevel>(level, true, out var parsedLevel))
            {
                ErrorLevel = parsedLevel;
            }

            var count = Error_GetErrorReferencesCount(prt);
            ErrorReferences ??= new List<ErrorReference>();
            ErrorReferences.Clear();
            for (var i = 0; i < count; i++)
            {
                var referencePtr = Error_GetErrorReferenceAt(prt, i);
                if (referencePtr == IntPtr.Zero)
                {
                    continue;
                }
                var reference = new ErrorReference();
                reference.GetDataWrapper(referencePtr);
                ErrorReferences.Add(reference);
            }
        }
    }

    public class ErrorReference: VDA5050MessageBase
    {
        private const string Lib = "libVDAWrapper.so";

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr ErrorReference_GetReferenceKey(IntPtr reference);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr ErrorReference_GetReferenceValue(IntPtr reference);

        public string ReferenceKey { get; set; }
        public string ReferenceValue { get; set; }

        public override void CreateWrapper()
        {
            // ErrorReference is a nested "get-only" structure (read from Error wrapper).
        }

        public override void GetDataWrapper(IntPtr prt)
        {
            ReferenceKey = VDA5050MessageBase.PtrToString(ErrorReference_GetReferenceKey(prt)) ?? "";
            ReferenceValue = VDA5050MessageBase.PtrToString(ErrorReference_GetReferenceValue(prt)) ?? "";
        }
    }

    public enum ErrorLevel
    {
        WARNING,
        FATAL
    }
}
